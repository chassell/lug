
#include "lug/lug.hpp"

#include <vector>
#include <string>
#include <memory>

#define INPUT_DEBUG  ( sr < input_.size() && input_[sr] >= ' ' ? ( "'"s + std::string(1,input_[sr]) + "'"s ) : "$" )

#define PE_DEBUG(info,pe)  { std::cerr << info << " -- @op:" << static_cast<int>(op) << " in+" << sr << ":" << INPUT_DEBUG << " set:" << lug::unicode::to_string(pe,str) << std::endl; }
#define OP_DEBUG(info)  { std::cerr << info << " -- @op:" << static_cast<int>(op) << " in+" << sr << ":" << INPUT_DEBUG << "" << std::endl; }
#define STR_DEBUG(info,str)  { std::cerr << info << " -- @op:" << static_cast<int>(op) << " in+" << sr << ":" << INPUT_DEBUG << ( str.empty() ? "" : " str:"s + std::string(str) ) << std::endl; }

template <class RuneSet>
auto lug::add_rune_range(RuneSet&& runes, directives mode, char32_t first, char32_t last) -> RuneSet &&
{
	if (first > last)
		throw bad_character_range{};
	if ((mode & directives::caseless) != directives::none)
		unicode::push_casefolded_range(runes, first, last);
	else
		unicode::push_range(runes, first, last);
	return ::std::move(runes);
}

template auto lug::add_rune_range(lug::unicode::rune_set&& runes, directives mode, char32_t first, char32_t last) -> lug::unicode::rune_set &&;

auto lug::instruction::decode(std::vector<instruction> const& code, std::ptrdiff_t& pc)
{
        auto pf = code[pc++].pf;
        auto imm = pf.val;
        auto off = (pf.aux & operands::off) != operands::none ? code[pc++].off : 0;
        std::string_view str;
        if ((pf.aux & operands::str) != operands::none) {
                str = std::string_view{code[pc].str.data(), static_cast<unsigned int>((imm & 0xff) + 1)};
                pc += ((imm & 0xff) + 4) >> 2;
                imm >>= 8;
        }
        return std::make_tuple(pf.op, imm, off, str);
}

std::ptrdiff_t lug::instruction::length(prefix pf) noexcept
{
        std::ptrdiff_t len = 1;
        if ((pf.aux & operands::off) != operands::none)
                ++len;
        if ((pf.aux & operands::str) != operands::none)
                len += static_cast<std::ptrdiff_t>(((pf.val & 0xff) >> 2) + 1);
        return len;
}

bool lug::parser::available(std::size_t sr, std::size_t sn)
{
    do {
            if (sn <= input_.size() - sr)
                    return true;
            if (sr < input_.size())
                    return false;
    } while (read_more());
    return false;
}

bool lug::parser::read_more()
{
    detail::reentrancy_sentinel<reenterant_read_error> guard{reading_};
    std::string text;
    while (!sources_.empty() && text.empty()) {
            bool more = sources_.back()(text);
            input_.insert(input_.end(), text.begin(), text.end());
            if (!more)
                    sources_.pop_back();
    }
    return !text.empty();
}

int lug::parser::casefold_compare(std::size_t sr, std::size_t sn, std::string_view str)
{
    auto& subject = casefolded_subjects_[sr];
    if (subject.size() < sn)
            subject = utf8::tocasefold(std::string_view{input_.data() + sr, sn});
    return subject.compare(0, sn, str);
}

void lug::parser::accept(std::size_t sr, std::size_t mr, std::size_t rc, std::ptrdiff_t pc)
{
    registers_ = {sr, (std::max)(mr, sr), rc, pc, 0};
    auto const& actions = grammar_.program().actions;
    auto const& captures = grammar_.program().captures;
    environment_.start_accept(*this);
    for (auto& response : responses_) {
            if (prune_depth_ <= response.call_depth)
                    continue;
            prune_depth_ = max_call_depth, call_depth_ = response.call_depth;
            if (response.range.index < max_size)
                    captures[response.action_index](environment_, syntax{*this, response.range});
            else
                    actions[response.action_index](environment_);
    }
    environment_.end_accept();
}

auto lug::parser::drain() -> decltype(std::declval<parser_registers>().as_tuple())
{
    origin_ = position_at(registers_.sr);
    input_.erase(0, registers_.sr);
    casefolded_subjects_.clear();
    positions_.clear();
    responses_.clear();
    registers_.mr -= registers_.sr;
    registers_.sr = 0, registers_.rc = 0;
    cut_deferred_ = false, cut_frame_ = stack_frames_.size();
    return registers_.as_tuple();
}

void lug::parser::pop_responses_after(std::size_t n)
{
    if (n < responses_.size())
            responses_.resize(n);
}

auto lug::parser::drop_responses_after(std::size_t n) -> std::vector<semantic_response>
{
    std::vector<semantic_response> dropped;
    if (n < responses_.size()) {
            dropped.assign(responses_.begin() + n, responses_.end());
            responses_.resize(n);
    }
    return dropped;
}

auto lug::parser::restore_responses_after(std::size_t n, std::vector<semantic_response> const& restore) -> size_t
{
    pop_responses_after(n);
    responses_.insert(responses_.end(), restore.begin(), restore.end());
    return responses_.size();
}

auto lug::parser::push_response(std::size_t depth, std::size_t action_index, syntax_range range) -> size_t
{
    responses_.push_back({static_cast<unsigned short>(depth), static_cast<unsigned short>(action_index), range});
    return responses_.size();
}

void lug::program::concatenate(program const& src)
{
    instructions.reserve(detail::checked_add<program_limit_error>(instructions.size(), src.instructions.size()));
    for (auto i = src.instructions.begin(), j = i, e = src.instructions.end(); i != e; i = j) {
            instruction instr = *i;
            std::size_t val;
            switch (instr.pf.op) {
                    case opcode::match_set: val = detail::push_back_unique(runesets, src.runesets[instr.pf.val]); break;
                    case opcode::predicate: val = predicates.size(); predicates.push_back(src.predicates[instr.pf.val]); break;
                    case opcode::action: val = actions.size(); actions.push_back(src.actions[instr.pf.val]); break;
                    case opcode::end: val = captures.size(); captures.push_back(src.captures[instr.pf.val]); break;
                    default: val = (std::numeric_limits<std::size_t>::max)(); break;
            }
            if (val != (std::numeric_limits<std::size_t>::max)()) {
                    detail::assure_in_range<resource_limit_error>(val, 0u, (std::numeric_limits<unsigned short>::max)());
                    instr.pf.val = static_cast<unsigned short>(val);
            }
            j = std::next(i, instruction::length(instr.pf));
            instructions.push_back(instr);
            instructions.insert(instructions.end(), i + 1, j);
    }
    mandate = (mandate & ~directives::eps) | (mandate & src.mandate & directives::eps);
}

lug::encoder& lug::encoder::do_call(rule const* r, program const* p, std::ptrdiff_t off, unsigned short prec)
{
        auto callee_mode = mode_.back();
        skip(p->mandate ^ directives::eps, directives::noskip);
        do_add_callee(r, p, length(), callee_mode);
        return encode(opcode::call, off, immediate{prec});
}

lug::encoder& lug::encoder::do_match(opcode op, std::string_view sequence)
{
        while (sequence.size() > instruction::maxstrlen) {
                std::string_view subsequence = sequence.substr(0, instruction::maxstrlen);
                while (!subsequence.empty() && !utf8::is_lead(subsequence.back()))
                        subsequence.remove_suffix(1);
                subsequence.remove_suffix(!subsequence.empty());
                encode(op, subsequence);
                sequence.remove_prefix(subsequence.size());
        }
        return encode(op, sequence);
}

lug::encoder& lug::encoder::dpop(directives relay)
{
        auto prev = detail::pop_back(mode_), next = (mode_.back() & ~relay) | (prev & relay);
        if ((next & directives::postskip) == directives::none && (prev & (directives::lexeme | directives::noskip | directives::postskip)) == directives::postskip)
                do_skip();
        mode_.back() = next;
        return *this;
}

lug::encoder& lug::encoder::skip(directives callee_mandate, directives callee_skip)
{
        auto mode = mode_.back();
        if (mandate_ == directives::none)
                mandate_ = (mode & (directives::caseless | directives::lexeme | directives::noskip)) | directives::eps;
        if ((((mode | callee_mandate)) & (callee_skip | directives::preskip)) == directives::preskip)
                do_skip();
        mode_.back() = mode & ~(callee_mandate & directives::eps);
        return *this;
}

lug::encoder& lug::encoder::call(rule const& r, unsigned short prec, bool allow_inlining)
{
        if (auto const& p = r.program_; allow_inlining && prec <= 0 && !r.currently_encoding_ && r.callees_.empty() && !p.instructions.empty() &&
                                p.instructions.size() <= 8 && p.predicates.size() <= 1 && p.actions.size() <= 1 && p.captures.size() <= 1)
                return skip(p.mandate, directives::noskip).append(p);
        return do_call(&r, &r.program_, 0, prec);
}

lug::encoder& lug::encoder::encode(opcode op, std::string_view subsequence, immediate imm)
{
        if (!subsequence.empty()) {
                detail::assure_in_range<resource_limit_error>(static_cast<unsigned short>(imm), 0u, instruction::maxstrlen - 1);
                detail::assure_in_range<resource_limit_error>(subsequence.size(), 1u, instruction::maxstrlen);
                do_append(instruction{op, operands::str, static_cast<immediate>((static_cast<unsigned short>(imm) << 8) | (subsequence.size() - 1))});
                do {
                        do_append(instruction{subsequence});
                        subsequence.remove_prefix((std::min)(std::size_t{4}, subsequence.size()));
                } while (!subsequence.empty());
        }
        return *this;
}

lug::encoder& lug::encoder::match(std::string_view subject)
{
        skip(!subject.empty() ? directives::eps : directives::none);
        if ((mode() & directives::caseless) != directives::none)
                return do_match(opcode::match_casefold, utf8::tocasefold(subject));
        else
                return do_match(opcode::match, subject);
}

void lug::basic_regular_expression::generator::bracket_class(std::string_view s)
{
    if (auto c = unicode::stoctype(s); c.has_value())
            classes |= c.value();
    else
            throw bad_character_class{};
}

void lug::basic_regular_expression::generator::bracket_range(std::string_view s)
{
        bracket_range(s.substr(0, s.find('-')), s.substr(s.find('-') + 1));
}

void lug::basic_regular_expression::generator::bracket_range(std::string_view a, std::string_view b)
{
        add_rune_range(std::ref(runes), encoder.mode(),
                utf8::decode_rune(std::begin(a), std::end(a)).second,
                utf8::decode_rune(std::begin(b), std::end(b)).second);
}

void lug::basic_regular_expression::generator::bracket_commit()
{
        runes = unicode::sort_and_optimize(std::move(runes));
        if (!runes.empty() && classes == unicode::ctype::none) {
                if (circumflex)
                        runes = unicode::negate(runes);
                encoder.match(std::move(runes));
        } else {
                if (circumflex)
                        encoder.encode(opcode::choice, 3 + (!runes.empty() ? 1 : 0) + (classes != unicode::ctype::none ? 1 : 0));
                if (!runes.empty())
                        encoder.match(std::move(runes));
                if (classes != unicode::ctype::none)
                        encoder.match_any(classes);
                if (circumflex)
                        encoder.encode(opcode::commit, 0).encode(opcode::fail).match_any();
        }
        runes.clear(), classes = unicode::ctype::none, circumflex = false;
}

lug::grammar lug::start(rule const& start_rule)
{
	program grprogram;
	program_callees grcallees;
	std::unordered_map<program const*, std::ptrdiff_t> addresses;
	std::vector<std::pair<program const*, std::ptrdiff_t>> calls;
	std::unordered_set<program const*> left_recursive;
	std::vector<std::pair<std::vector<std::pair<rule const*, bool>>, program const*>> unprocessed;
	program_encoder{grprogram, grcallees, directives::eps | directives::preskip}.call(start_rule, 1, false).encode(opcode::accept_final);
	calls.emplace_back(&start_rule.program_, std::get<2>(grcallees.back()));
	unprocessed.emplace_back(std::vector<std::pair<rule const*, bool>>{{&start_rule, false}}, &start_rule.program_);
	do {
		auto [callstack, subprogram] = detail::pop_back(unprocessed);
		auto const address = static_cast<std::ptrdiff_t>(grprogram.instructions.size());
		if (addresses.emplace(subprogram, address).second) {
			grprogram.concatenate(*subprogram);
			grprogram.instructions.emplace_back(opcode::ret, operands::none, immediate{0});
			if (auto top_rule = callstack.back().first; top_rule) {
				for (auto [callee_rule, callee_program, instr_offset, mode] : top_rule->callees_) {
					// Make a local copy of callee_rule that can be lambda-captured
					auto rule = callee_rule;

					calls.emplace_back(callee_program, address + instr_offset);
					if (callee_rule && (mode & directives::eps) != directives::none && detail::escaping_find_if(
							callstack.crbegin(), callstack.crend(), [rule](auto& caller) {
								return caller.first == rule ? 1 : (caller.second ? 0 : -1); }) != callstack.crend()) {
						left_recursive.insert(callee_program);
					} else {
						auto callee_callstack = callstack;
						callee_callstack.emplace_back(callee_rule, (mode & directives::eps) != directives::none);
						unprocessed.emplace_back(std::move(callee_callstack), callee_program);
					}
				}
			}
		}
	} while (!unprocessed.empty());
	for (auto [subprogram, instr_addr] : calls) {
		if (auto& iprefix = grprogram.instructions[instr_addr]; iprefix.pf.op == opcode::call)
			iprefix.pf.val = left_recursive.count(subprogram) != 0 ? (iprefix.pf.val != 0 ? iprefix.pf.val : 1) : 0;
		auto& ioffset = grprogram.instructions[instr_addr + 1];
		auto const rel_addr = ioffset.off + addresses[subprogram] - (instr_addr + 2);
		detail::assure_in_range<program_limit_error>(rel_addr, std::numeric_limits<int>::lowest(), (std::numeric_limits<int>::max)());
		ioffset.off = static_cast<int>(rel_addr);
	}
	grammar::implicit_space = language::operator*(language::space);
	return grammar{std::move(grprogram)};
}

lug::syntax_position const& lug::parser::position_at(std::size_t index)
{
    auto pos = std::lower_bound(std::begin(positions_), std::end(positions_), index, [](auto& x, auto& y) { return x.first < y; });
    if (pos != std::end(positions_) && index == pos->first)
            return pos->second;
    std::size_t startindex = 0;
    syntax_position position = origin_;
    if (pos != std::begin(positions_)) {
            auto prevpos = std::prev(pos);
            startindex = prevpos->first;
            position = prevpos->second;
    }
    auto first = std::next(std::begin(input_), startindex);
    auto const last = std::next(std::begin(input_), index);
    char32_t rune, prevrune = U'\0';
    for (auto curr = first, next = curr; curr < last; curr = next, prevrune = rune) {
            std::tie(next, rune) = utf8::decode_rune(curr, last);
            if ((unicode::query(rune).properties() & unicode::ptype::Line_Ending) != unicode::ptype::None && (prevrune != U'\r' || rune != U'\n')) {
                    position.column = 1, ++position.line;
                    first = next;
            }
    }
    for (auto curr = first, next = curr; curr < last; curr = next) {
            std::tie(next, rune) = utf8::decode_rune(curr, last);
            if (rune != U'\t') {
                    position.column += unicode::ucwidth(rune);
            } else {
                    auto oldcolumn = position.column;
                    auto newcolumn = oldcolumn + environment_.tab_width();
                    auto alignedcolumn = newcolumn - ((newcolumn - 1) % environment_.tab_alignment());
                    position.column = (std::max)((std::min)(newcolumn, alignedcolumn), oldcolumn);
            }
    }
    return positions_.insert(pos, std::make_pair(index, position))->second;
}
        
template <lug::opcode Opcode>
bool lug::parser::commit(std::size_t& sr, std::size_t& rc, std::ptrdiff_t& pc, int off)
{
        if (stack_frames_.empty() || stack_frames_.back() != stack_frame_type::backtrack)
                return false;
        if constexpr (Opcode == opcode::commit_partial) {
                detail::make_tuple_view<0, 1>(backtrack_stack_.back()) = {sr, rc};
        } else {
                detail::ignore(sr, rc);
                if constexpr (Opcode == opcode::commit_back)
                        sr = std::get<0>(backtrack_stack_.back());
                pop_stack_frame(backtrack_stack_);
        }
        pc += off;
        return true;
}

template bool lug::parser::commit<lug::opcode::commit>(std::size_t& sr, std::size_t& rc, std::ptrdiff_t& pc, int off);
template bool lug::parser::commit<lug::opcode::commit_back>(std::size_t& sr, std::size_t& rc, std::ptrdiff_t& pc, int off);
template bool lug::parser::commit<lug::opcode::commit_partial>(std::size_t& sr, std::size_t& rc, std::ptrdiff_t& pc, int off);

bool lug::parser::parse()
{
    detail::reentrancy_sentinel<reenterant_parse_error> guard{parsing_};
    program const& prog = grammar_.program();
    if (prog.instructions.empty())
            throw bad_grammar{};
    auto [sr, mr, rc, pc, fc] = drain();
    bool result = false, done = false;
    prune_depth_ = max_call_depth, call_depth_ = 0;
    pc = 0, fc = 0;
    while (!done) {
            auto [op, imm, off, str] = instruction::decode(prog.instructions, pc);

            auto pe = static_cast<unicode::property_enum>(imm);
            auto s = str;    // local copy for lambda capture
            auto immstr = std::to_string(imm);

            switch (op) {
                    case opcode::match: {
                            if (!match_sequence(sr, str, [this](auto i, auto n, auto s) { return input_.compare(i, n, s) == 0; }))
                                {
                                    STR_DEBUG("!match",str);
                                    goto failure;
                                }
                                STR_DEBUG("match",str);
                    } break;
                    case opcode::match_casefold: {
                            if (!match_sequence(sr, str, [this](auto i, auto n, auto s) { return casefold_compare(i, n, s) == 0; }))
                                {
                                    STR_DEBUG("!match-case",str);
                                    goto failure;
                                }
                            STR_DEBUG("match-case",str);
                    } break;
                    case opcode::match_any: {
                            if (!match_single(sr, []{ return true; }))
                                {
                                    OP_DEBUG("!any");
                                    goto failure;
                                }
                            OP_DEBUG("any");
                    } break;
                    case opcode::match_any_of: {
                            if (!match_single(sr, [pe, s](auto const& r) { return unicode::any_of(r, pe, s); }))
                                {
                                    PE_DEBUG("!any",pe);
                                    goto failure;
                                }
                                PE_DEBUG("any",pe);
                    } break;
                    case opcode::match_all_of: {
                            if (!match_single(sr, [pe, s](auto const& r) { return unicode::all_of(r, pe, s); }))
                                {
                                    PE_DEBUG("!all",pe);
                                    goto failure;
                                }
                                PE_DEBUG("all",pe);
                    } break;
                    case opcode::match_none_of: {
                            if (!match_single(sr, [pe, s](auto const& r) { return unicode::none_of(r, pe, s); }))
                                {
                                    PE_DEBUG("!none",pe);
                                    goto failure;
                                }
                                PE_DEBUG("none",pe);
                    } break;
                    case opcode::match_set: {
                            auto imm_copy = imm;
                            if (!match_single(sr, [&runes = prog.runesets[imm_copy]](char32_t rune) {
                                            auto interval = std::lower_bound(runes.begin(), runes.end(), rune, [](auto& x, auto& y) { return x.second < y; });
                                            return interval != runes.end() && interval->first <= rune && rune <= interval->second; }))
                                {
                                    STR_DEBUG("!set",immstr);
                                    goto failure;
                                }
                                STR_DEBUG("set",immstr);
                    } break;
                    case opcode::match_eol: {
                            if (!match_single(sr, [](auto curr, auto last, auto& next, char32_t rune) {
                                            if (curr == next || (unicode::query(rune).properties() & unicode::ptype::Line_Ending) == unicode::ptype::None)
                                                    return false;
                                            if (U'\r' == rune)
                                                    if (auto [next2, rune2] = utf8::decode_rune(next, last); next2 != next && rune2 == U'\n')
                                                            next = next2;
                                            return true; }))
                                {
                                    OP_DEBUG("!eol");
                                    goto failure;
                                }
                                OP_DEBUG("eol");
                    } break;
                    case opcode::choice: {
                            OP_DEBUG("try");
                            stack_frames_.push_back(stack_frame_type::backtrack);
                            backtrack_stack_.emplace_back(sr - imm, rc, pc + off);
                    } break;
                    case opcode::commit: {
                            if (!commit<opcode::commit>(sr, rc, pc, off))
                                {
                                    OP_DEBUG("!commit");
                                    goto failure;
                                }
                                OP_DEBUG("commit");
                    } break;
                    case opcode::commit_back: {
                            if (!commit<opcode::commit_back>(sr, rc, pc, off))
                                {
                                    OP_DEBUG("!commit_back");
                                    goto failure;
                                }
                                OP_DEBUG("commit_back");
                    } break;
                    case opcode::commit_partial: {
                            if (!commit<opcode::commit_partial>(sr, rc, pc, off))
                                {
                                    OP_DEBUG("!commit_partial");
                                    goto failure;
                                }
                                OP_DEBUG("commit_partial");
                    } break;
                    case opcode::jump: {
                            pc += off;
                    } break;
                    case opcode::call: {
                            if (imm != 0) {
                                    auto memo = detail::escaping_find_if(lrmemo_stack_.crbegin(), lrmemo_stack_.crend(),
                                                    [sr = sr, pca = pc + off](auto const& m) -> int { return ( m.srr == sr && m.pca == pca ? 1 : (m.srr < sr ? 0 : -1) ); });
                                    if (memo != lrmemo_stack_.crend()) {
                                            if (memo->sra == lrfailcode || imm < memo->prec)
                                                {
                                                    goto failure;
                                                    STR_DEBUG("!call-failed",immstr);
                                                }
                                            sr = memo->sra, rc = restore_responses_after(rc, memo->responses);
                                            STR_DEBUG("!call-failed-continue",immstr);
                                            continue;
                                    }
                                    stack_frames_.push_back(stack_frame_type::lrcall);
                                    lrmemo_stack_.push_back({sr, lrfailcode, imm, pc, pc + off, rc, std::vector<semantic_response>{}});
                            } else {
                                    stack_frames_.push_back(stack_frame_type::call);
                                    call_stack_.push_back(pc);
                            }
                            pc += off;
                            STR_DEBUG("call",immstr);
                    } break;
                    case opcode::ret: {
                            if (stack_frames_.empty())
                                {
                                    OP_DEBUG("ret-fail");
                                    goto failure;
                                }
                            switch (stack_frames_.back()) {
                                    case stack_frame_type::call: {
                                            pc = call_stack_.back();
                                            pop_stack_frame(call_stack_);
                                            OP_DEBUG("ret");
                                    } break;
                                    case stack_frame_type::lrcall: {
                                            auto& memo = lrmemo_stack_.back();
                                            if (memo.sra == lrfailcode || sr > memo.sra) {
                                                    memo.sra = sr, memo.responses = drop_responses_after(memo.rcr);
                                                    sr = memo.srr, pc = memo.pca, rc = memo.rcr;
                                                    OP_DEBUG("ret-lrcall-fail-continue");
                                                    continue;
                                            }
                                            sr = memo.sra, pc = memo.pcr, rc = restore_responses_after(memo.rcr, memo.responses);
                                            pop_stack_frame(lrmemo_stack_, sr, mr, rc, pc);
                                            OP_DEBUG("ret-lrcall");
                                    } break;
                                    default: 
                                    OP_DEBUG("ret-deflt-fail");
                                    goto failure;
                            }
                    } break;
                    case opcode::fail: {
                            OP_DEBUG("fail-fail");
                            fc = imm;
                    failure:
                            for (mr = (std::max)(mr, sr), ++fc; fc > 0; --fc) {
                                    if (done = cut_frame_ >= stack_frames_.size(); done) {
                                            registers_ = {sr, mr, rc, pc, 0};
                                            break;
                                    }
                                    switch (stack_frames_.back()) {
                                            case stack_frame_type::backtrack: {
                                                    std::tie(sr, rc, pc) = backtrack_stack_.back();
                                                    pop_stack_frame(backtrack_stack_);
                                            } break;
                                            case stack_frame_type::call: {
                                                    pop_stack_frame(call_stack_), ++fc;
                                            } break;
                                            case stack_frame_type::capture: {
                                                    pop_stack_frame(capture_stack_, sr, mr, rc, pc), ++fc;
                                            } break;
                                            case stack_frame_type::lrcall: {
                                                    if (auto& memo = lrmemo_stack_.back(); memo.sra != lrfailcode)
                                                            sr = memo.sra, pc = memo.pcr, rc = restore_responses_after(memo.rcr, memo.responses);
                                                    else
                                                            ++fc;
                                                    pop_stack_frame(lrmemo_stack_, sr, mr, rc, pc);
                                            } break;
                                            default: break;
                                    }
                            }
                            pop_responses_after(rc);
                    } break;
                    case opcode::accept: {
                            OP_DEBUG("accept");
                            if (cut_deferred_ = !capture_stack_.empty() || !lrmemo_stack_.empty(); !cut_deferred_) {
                                    accept(sr, mr, rc, pc);
                                    std::tie(sr, mr, rc, pc, std::ignore) = drain();
                            }
                    } break;
                    case opcode::accept_final: {
                            OP_DEBUG("accept-final");
                            accept(sr, mr, rc, pc);
                            result = done = true;
                    } break;
                    case opcode::predicate: {
                            registers_ = {sr, (std::max)(mr, sr), rc, pc, 0};
                            bool accepted = prog.predicates[imm](*this);
                            std::tie(sr, mr, rc, pc, fc) = registers_.as_tuple();
                            pop_responses_after(rc);
                            if (!accepted)
                                {
                                    goto failure;
                                }
                    } break;
                    case opcode::action: {
                            OP_DEBUG("action");
                            rc = push_response(call_stack_.size() + lrmemo_stack_.size(), imm);
                    } break;
                    case opcode::begin: {
                            OP_DEBUG("begin");
                            stack_frames_.push_back(stack_frame_type::capture);
                            capture_stack_.push_back(static_cast<subject_location>(sr));
                    } break;
                    case opcode::end: {
                            if (stack_frames_.empty() || stack_frames_.back() != stack_frame_type::capture)
                                {
                                    OP_DEBUG("end-fail");
                                    goto failure;
                                }
                            OP_DEBUG("end");
                            auto sr0 = static_cast<std::size_t>(capture_stack_.back()), sr1 = sr;
                            pop_stack_frame(capture_stack_, sr, mr, rc, pc);
                            if (sr0 > sr1)
                                {
                                    OP_DEBUG("pop-fail");
                                    goto failure;
                                }
                            OP_DEBUG("push-response");
                            rc = push_response(call_stack_.size() + lrmemo_stack_.size(), imm, {sr0, sr1 - sr0});
                    } break;
                    default: registers_ = {sr, (std::max)(mr, sr), rc, pc, 0}; throw bad_opcode{};
            }
    }
    return result;
}

lug::grammar lug::basic_regular_expression::make_grammar()
{
	using namespace language;
	auto old_implicit_space = grammar::implicit_space;
	grammar::implicit_space = nop;
	rule Empty = eps                                    <[](generator& g) { g.encoder.match_eps(); };
	rule Dot = chr('.')                                 <[](generator& g) { g.encoder.match_any(); };
	rule Element = any > chr('-') > !chr(']') > any     <[](generator& g, csyntax& x) { g.bracket_range(x.capture()); }
	    | str("[:") > +(!chr(':') > any) > str(":]")    <[](generator& g, csyntax& x) { g.bracket_class(x.capture().substr(2, x.range().size - 4)); }
	    | any                                           <[](generator& g, csyntax& x) { g.bracket_range(x.capture(), x.capture()); };
	rule Bracket = chr('[') > ~(chr('^')                <[](generator& g) { g.circumflex = true; })
	    > Element > *(!chr(']') > Element) > chr(']')   <[](generator& g) { g.bracket_commit(); };
	rule Sequence = +(!(chr('.') | chr('[')) > any)     <[](generator& g, csyntax& x) { g.encoder.match(x.capture()); };
	grammar grmr = start((+(Dot | Bracket | Sequence) | Empty) > eoi);
	grammar::implicit_space = old_implicit_space;
	return grmr;
}

void lug::basic_regular_expression::operator()(encoder& d) const
{
	if (program_->instructions.empty()) {
		static grammar const grmr = make_grammar();
		generator genr(*this, d.mode() & directives::caseless);
		if (!parse(expression_, grmr, genr))
			throw bad_string_expression{};
	}
	d.skip((program_->mandate & directives::eps) ^ directives::eps).append(*program_);
}

using lug::directives;

template struct lug::directive_modifier<directives::none, directives::none, directives::none>;
template struct lug::directive_modifier<directives::none, directives::none, directives::eps>;
template struct lug::directive_modifier<directives::postskip, directives::none, directives::eps>;
template struct lug::directive_modifier<directives::preskip, directives::postskip, directives::eps>;
template struct lug::directive_modifier<directives::none, directives::caseless, directives::eps>;
template struct lug::directive_modifier<directives::caseless, directives::none, directives::eps>;
template struct lug::directive_modifier<directives::lexeme, directives::noskip, directives::eps>;
template struct lug::directive_modifier<directives::lexeme | directives::noskip, directives::none, directives::eps>;
template struct lug::directive_modifier<directives::none, directives::lexeme | directives::noskip, directives::eps>;
