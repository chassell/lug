// lug - Embedded DSL for PE grammar parser combinators in C++
// Copyright (c) 2017 Jesse W. Towner
// See LICENSE.md file for license details

#pragma once

#include "lug/error.hpp"
#include "lug/utf8.hpp"
#include <any>
#include <iostream>
#include <numeric>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>

extern template class std::unordered_map<std::size_t, std::string>;
extern template class std::unordered_map<unsigned short, double>;
extern template class std::unordered_map<unsigned short, int>;
extern template class std::vector<std::any>;

using namespace std::literals::string_literals;
namespace lug
{

struct program;
class rule;
class grammar;
class encoder;
class rule_encoder;
class parser;
class syntax;
class environment;

struct syntax_position { std::size_t column, line; };
struct syntax_range { std::size_t index, size; };
struct semantic_response { unsigned short call_depth, action_index; syntax_range range; };
using syntactic_capture = std::function<void(environment&, syntax const&)>;
using semantic_action = std::function<void(environment&)>;
using semantic_predicate = std::function<bool(parser&)>;

template <class E> constexpr bool is_callable_v =
	std::is_same_v<grammar, std::decay_t<E>> ||
	std::is_same_v<rule, std::decay_t<E>> ||
	std::is_same_v<program, std::decay_t<E>>;

template <class E> constexpr bool is_predicate_v = std::is_invocable_r_v<bool, E, parser&> || std::is_invocable_r_v<bool, E>;
template <class E> constexpr bool is_proper_expression_v = std::is_invocable_v<E, encoder&>;
template <class E> constexpr bool is_string_expression_v = std::is_convertible_v<E, std::string>;
template <class E> constexpr bool is_expression_v = is_callable_v<E> || is_predicate_v<E> || is_proper_expression_v<E> || is_string_expression_v<E>;

grammar start(rule const& start_rule);

enum class opcode : unsigned char
{
	match,          match_casefold, match_any,      match_any_of,
	match_all_of,   match_none_of,  match_set,      match_eol,
	choice,         commit,         commit_back,    commit_partial,
	jump,           call,           ret,            fail,
	accept,         accept_final,   predicate,      action,
	begin,          end
};

enum class immediate : unsigned short {};
enum class operands : unsigned char { none = 0, off = 0x40, str = 0x80, is_bitfield_enum };

union instruction
{
	static constexpr std::size_t maxstrlen = 256;
	struct prefix { opcode op; operands aux; unsigned short val; } pf;
	int off{};
	std::array<char, 4> str;

	instruction(opcode op, operands aux, immediate imm) : pf{op, aux, static_cast<unsigned short>(imm)} {}
	instruction(std::ptrdiff_t o) : off{static_cast<int>(o)} { if (off != o) throw program_limit_error{}; }
	instruction(std::string_view s) { std::fill(std::copy_n(s.begin(), (std::min)(s.size(), std::size_t{4}), str.begin()), str.end(), char{0}); }
	instruction() = default;

	static auto decode(std::vector<instruction> const& code, std::ptrdiff_t& pc);

	static std::ptrdiff_t length(prefix pf) noexcept;

};


static_assert(sizeof(unicode::ctype) <= sizeof(immediate), "immediate must be large enough to hold unicode::ctype");
static_assert(sizeof(unicode::sctype) <= sizeof(immediate), "immediate must be large enough to hold unicode::sctype");
static_assert(sizeof(instruction) == sizeof(int), "expected instruction to be same size as int");
static_assert(sizeof(int) <= sizeof(std::ptrdiff_t), "expected int to be no larger than ptrdiff_t");

enum class directives : unsigned int { none = 0, caseless = 1, eps = 2, lexeme = 4, noskip = 8, preskip = 16, postskip = 32, is_bitfield_enum };
using program_callees = std::vector<std::tuple<lug::rule const*, lug::program const*, std::ptrdiff_t, directives>>;

struct program
{
	std::vector<instruction> instructions;
	std::vector<unicode::rune_set> runesets;
	std::vector<semantic_predicate> predicates;
	std::vector<semantic_action> actions;
	std::vector<syntactic_capture> captures;
	directives mandate{directives::eps};

	void concatenate(program const& src);

	void swap(program& p)
	{
		instructions.swap(p.instructions);
		runesets.swap(p.runesets);
		predicates.swap(p.predicates);
		actions.swap(p.actions);
		captures.swap(p.captures);
		std::swap(mandate, p.mandate);
	}
};

class rule
{
	friend class encoder;
	friend class rule_encoder;
	friend grammar start(rule const&);
	program program_;
	program_callees callees_;
	bool currently_encoding_{false};
public:
	rule() = default;
	template <class E, class = std::enable_if_t<is_expression_v<E>>> rule(E const& e);
	rule(rule const& r);
	rule(rule&& r) = default;
	rule& operator=(rule const& r) { rule{r}.swap(*this); return *this; }
	rule& operator=(rule&& r) = default;
	void swap(rule& r) { program_.swap(r.program_); callees_.swap(r.callees_); }
	auto operator[](unsigned short precedence) const noexcept;
};

class grammar
{
	friend grammar start(rule const&);
	lug::program program_;
	grammar(lug::program p) : program_{std::move(p)} {}
public:
	grammar() = default;
	void swap(grammar& g) { program_.swap(g.program_); }
	lug::program const& program() const noexcept { return program_; };
	static std::function<void(encoder&)> implicit_space;
};

class syntax
{
	lug::parser& parser_;
	syntax_range const range_;
public:
	syntax(lug::parser& p, syntax_range const& r) : parser_{p}, range_{r} {}
	lug::parser& parser() const noexcept { return parser_; }
	syntax_range range() const noexcept { return range_; }
	syntax_position const& start() const;
	syntax_position const& end() const;
	std::string_view capture() const;
};

class environment
{
	friend class lug::parser;

	lug::parser* parser_ = nullptr;
	std::vector<std::any> attributes_;
	unsigned int tab_width_ = 8;
	unsigned int tab_alignment_ = 8;

	virtual void on_accept_started() {}
	virtual void on_accept_ended() {}

	void start_accept(lug::parser& p)
	{
		if (parser_)
			throw reenterant_accept_error{};
		parser_ = &p;
		attributes_.clear();
		on_accept_started();
	}

	void end_accept()
	{
		on_accept_ended();
		parser_ = nullptr;
	}

public:
	virtual ~environment() = default;
	lug::parser& parser() { if (!parser_) throw accept_context_error{}; return *parser_; }
	lug::parser const& parser() const { if (!parser_) throw accept_context_error{}; return *parser_; }
	unsigned int tab_width() const { return tab_width_; }
	void tab_width(unsigned int w) { tab_width_ = w; }
	unsigned int tab_alignment() const { return tab_alignment_; }
	void tab_alignment(unsigned int a) { tab_alignment_ = a; }
	std::string_view match() const;
	syntax_position const& position_at(std::size_t index);
	unsigned short call_depth() const;
	unsigned short prune_depth() const;
	void escape();

	template <class T>
	void push_attribute(T&& x)
	{
		attributes_.emplace_back(std::in_place_type<T>, ::std::forward<T>(x));
	}

	template <class T, class... Args>
	void push_attribute(Args&&... args)
	{
		attributes_.emplace_back(std::in_place_type<T>, ::std::forward<Args>(args)...);
	}

	template <class T> T pop_attribute()
	{
		T r{::std::any_cast<T>(detail::pop_back(attributes_))};
		return r;
	}
};

template <class T>
class variable
{
	environment& environment_;
	std::unordered_map<unsigned short, T> state_;
public:
	variable(environment& e) : environment_{e} {}
	T* operator->() { return &state_[environment_.call_depth()]; }
	T const* operator->() const { return &state_[environment_.call_depth()]; }
	T& operator*() { return state_[environment_.call_depth()]; }
	T const& operator*() const { return state_[environment_.call_depth()]; }
};

class encoder
{
	directives mandate_;
	std::vector<directives> mode_;
	virtual void do_append(instruction) = 0;
	virtual void do_append(program const&) = 0;
	virtual immediate do_add_rune_set(unicode::rune_set) { return immediate{0}; }
	virtual immediate do_add_semantic_predicate(semantic_predicate) { return immediate{0}; }
	virtual immediate do_add_semantic_action(semantic_action) { return immediate{0}; }
	virtual immediate do_add_syntactic_capture(syntactic_capture) { return immediate{0}; }
	virtual void do_add_callee(rule const*, program const*, std::ptrdiff_t, directives) {}
	virtual bool do_should_evaluate_length() const noexcept { return true; }
	virtual std::ptrdiff_t do_length() const noexcept = 0;

protected:
	encoder& do_call(rule const* r, program const* p, std::ptrdiff_t off, unsigned short prec);

	encoder& do_match(opcode op, std::string_view sequence);

	template <class T>
	encoder& do_match_class(opcode op, T value)
	{
		constexpr auto penum = immediate{static_cast<unsigned short>(unicode::to_property_enum_v<std::decay_t<T>>)};
		return encode(op, detail::string_pack(value), penum);
	}

	void do_skip()
	{
		mode_.back() = (mode_.back() & ~(directives::preskip | directives::postskip)) | directives::lexeme | directives::noskip;
		grammar::implicit_space(*this);
	}

public:
	explicit encoder(directives initial) : mandate_{directives::none}, mode_{initial} {}
	virtual ~encoder() = default;
	template <class E> auto evaluate(E const& e) -> std::enable_if_t<is_expression_v<E>, encoder&>;
	template <class E> auto evaluate_length(E const& e) -> std::enable_if_t<is_expression_v<E>, std::ptrdiff_t>;
	encoder& dpsh(directives enable, directives disable) { directives prev = mode_.back(); mode_.push_back((prev & ~disable) | enable); return *this; }
	encoder& append(instruction instr) { do_append(instr); return *this; }
	encoder& append(program const& p) { do_append(p); return *this; }
	encoder& call(program const& p, unsigned short prec) { return do_call(nullptr, &p, 0, prec); }
	encoder& call(grammar const& g, unsigned short prec) { return do_call(nullptr, &g.program(), 3, prec); }
	encoder& encode(opcode op, immediate imm = immediate{0}) { return append(instruction{op, operands::none, imm}); }
	encoder& encode(opcode op, semantic_predicate p) { return append(instruction{op, operands::none, do_add_semantic_predicate(std::move(p))}); }
	encoder& encode(opcode op, semantic_action a) { return append(instruction{op, operands::none, do_add_semantic_action(std::move(a))}); }
	encoder& encode(opcode op, syntactic_capture c) { return append(instruction{op, operands::none, do_add_syntactic_capture(std::move(c))}); }
	encoder& encode(opcode op, std::ptrdiff_t off, immediate imm = immediate{0}) { return append(instruction{op, operands::off, imm}).append(instruction{off}); }
	std::ptrdiff_t length() const noexcept { return do_length(); }
	directives mandate() const noexcept { return (mandate_ & ~directives::eps) | mode_.back(); }
	directives mode() const noexcept { return mode_.back(); }
	encoder& match(unicode::rune_set runes) { return skip().encode(opcode::match_set, do_add_rune_set(std::move(runes))); }
	encoder& match_eps() { return skip(directives::lexeme).encode(opcode::match); }
	encoder& match_any() { return skip().encode(opcode::match_any); }
	template <class T, class = std::enable_if_t<unicode::is_property_enum_v<T>>>
	encoder& match_any(T properties) { return skip().do_match_class(opcode::match_any_of, properties); }
	template <class T, class = std::enable_if_t<unicode::is_property_enum_v<T>>>
	encoder& match_all(T properties) { return skip().do_match_class(opcode::match_all_of, properties); }
	template <class T, class = std::enable_if_t<unicode::is_property_enum_v<T>>>
	encoder& match_none(T properties) { return skip().do_match_class(opcode::match_none_of, properties); }

	encoder& dpop(directives relay);

	encoder& skip(directives callee_mandate = directives::eps, directives callee_skip = directives::lexeme);

	encoder& call(rule const& r, unsigned short prec, bool allow_inlining = true);

	encoder& encode(opcode op, std::string_view subsequence, immediate imm = immediate{0});

	encoder& match(std::string_view subject);
};

class instruction_length_evaluator final : public encoder
{
	std::ptrdiff_t length_;
	void do_append(instruction) override { length_ = detail::checked_add<program_limit_error>(length_, std::ptrdiff_t{1}); }
	void do_append(program const& p) override { length_ = detail::checked_add<program_limit_error>(length_, static_cast<std::ptrdiff_t>(p.instructions.size())); }
	bool do_should_evaluate_length() const noexcept override { return false; }
	std::ptrdiff_t do_length() const noexcept override { return length_; }
public:
	explicit instruction_length_evaluator(directives initial) : encoder{initial}, length_{0} {}
};

class program_encoder : public encoder
{
	program& program_;
	program_callees& callees_;
	std::ptrdiff_t do_length() const noexcept override final { return static_cast<std::ptrdiff_t>(program_.instructions.size()); }
	void do_append(instruction instr) override final { program_.instructions.push_back(instr); }
	void do_append(program const& p) override final { program_.concatenate(p); }
	void do_add_callee(rule const* r, program const* p, std::ptrdiff_t n, directives d) override final { callees_.emplace_back(r, p, n, d); }
	immediate do_add_rune_set(unicode::rune_set r) override final { return add_item(program_.runesets, std::move(r)); }
	immediate do_add_semantic_predicate(semantic_predicate p) override final { return add_item(program_.predicates, std::move(p)); }
	immediate do_add_semantic_action(semantic_action a) override final { return add_item(program_.actions, std::move(a)); }
	immediate do_add_syntactic_capture(syntactic_capture a) override final { return add_item(program_.captures, std::move(a)); }

	template <class Item>
	immediate add_item(std::vector<Item>& items, Item&& item)
	{
		detail::assure_in_range<resource_limit_error>(items.size(), 0u, (std::numeric_limits<unsigned short>::max)() - 1u);
		items.push_back(::std::forward<Item>(item));
		return static_cast<immediate>(items.size() - 1);
	}

public:
	program_encoder(program& p, program_callees& c, directives initial) : encoder{initial}, program_{p}, callees_{c} {}
	~program_encoder() { program_.mandate = mandate(); }
};

class rule_encoder final : public program_encoder
{
	rule& rule_;
public:
	explicit rule_encoder(rule& r) : program_encoder{r.program_, r.callees_, directives::eps}, rule_{r} { rule_.currently_encoding_ = true; }
	~rule_encoder() override { rule_.currently_encoding_ = false; }
};

template <class RuneSet>
auto add_rune_range(RuneSet&& runes, directives mode, char32_t first, char32_t last) -> RuneSet &&;

class basic_regular_expression
{
	std::string const expression_;
	std::shared_ptr<program> const program_;

	static grammar make_grammar();

	struct generator : environment
	{
		basic_regular_expression const& owner;
		program_callees callees;
		program_encoder encoder;
		bool circumflex = false;
		unicode::ctype classes = unicode::ctype::none;
		unicode::rune_set runes;

		generator(basic_regular_expression const& se, directives mode)
			: owner{se}, encoder{*se.program_, callees, mode | directives::eps | directives::lexeme} {}

		void bracket_class(std::string_view s);

		void bracket_range(std::string_view s);

		void bracket_range(std::string_view a, std::string_view b);

		void bracket_commit();
	};

public:
	explicit basic_regular_expression(std::string_view e) : expression_{e}, program_{std::make_shared<program>()} {}
	void operator()(encoder& d) const;
};

class string_expression
{
	std::string_view const expression_;
public:
	explicit string_expression(std::string_view e) : expression_{e} {}
	void operator()(encoder& d) const { d.match(expression_); }
};

template <class T, class E = std::remove_cv_t<std::remove_reference_t<T>>, class = std::enable_if_t<is_proper_expression_v<E>>>
constexpr T const& make_expression(T const& t)
{
	return t;
}

template <class T, class E = std::remove_cv_t<std::remove_reference_t<T>>, class = std::enable_if_t<!is_proper_expression_v<E>>>
constexpr auto make_expression(T&& t)
{
	static_assert(is_expression_v<E>, "T must be an expression type");
	if constexpr (is_callable_v<E>)
		return [&target = t](encoder& d) { d.call(target, 0); };
	else if constexpr (std::is_invocable_r_v<bool, E, parser&>)
		return [p = semantic_predicate{t}](encoder& d) { d.encode(opcode::predicate, p); };
	else if constexpr (std::is_invocable_r_v<bool, E>)
		return [p = semantic_predicate{[a = E{t}](parser&) { return a(); }}](encoder& d) { d.encode(opcode::predicate, p); };
	else if constexpr (is_string_expression_v<E>)
		return string_expression{t};
}

template <class E>
inline auto encoder::evaluate(E const& e) -> std::enable_if_t<is_expression_v<E>, encoder&>
{
	make_expression(e)(*this);
	return *this;
}

template <class E>
inline auto encoder::evaluate_length(E const& e) -> std::enable_if_t<is_expression_v<E>, std::ptrdiff_t>
{
	return do_should_evaluate_length() ? instruction_length_evaluator{mode()}.evaluate(e).length() : 0;
}

template <class E, class>
inline rule::rule(E const& e)
{
	rule_encoder{*this}.evaluate(e);
}

inline rule::rule(rule const& r)
{
	rule_encoder{*this}.call(r, 1);
}

inline auto rule::operator[](unsigned short precedence) const noexcept
{
	return [&target = *this, precedence](encoder& d){ d.call(target, precedence); };
}

template <directives EnableMask, directives DisableMask, directives RelayMask>
struct directive_modifier
{
	template <class E, class = std::enable_if_t<is_expression_v<E>>>
	auto operator[](E const& e) const
	{
		return [e = make_expression(e)](encoder& d) { d.dpsh(EnableMask, DisableMask).evaluate(e).dpop(RelayMask); };
	}
};

extern template struct directive_modifier<directives::caseless, directives::none, directives::eps>;
extern template struct directive_modifier<directives::lexeme, directives::noskip, directives::eps>;
extern template struct directive_modifier<directives::lexeme | directives::noskip, directives::none, directives::eps>;
extern template struct directive_modifier<directives::none, directives::caseless, directives::eps>;
extern template struct directive_modifier<directives::none, directives::none, directives::eps>;
extern template struct directive_modifier<directives::none, directives::none, directives::none>;
extern template struct directive_modifier<directives::postskip, directives::none, directives::eps>;
extern template struct directive_modifier<directives::preskip, directives::postskip, directives::eps>;

constexpr auto matches_eps = directive_modifier<directives::none, directives::none, directives::none>{};
constexpr auto relays_eps = directive_modifier<directives::none, directives::none, directives::eps>{};
constexpr auto skip_after = directive_modifier<directives::postskip, directives::none, directives::eps>{};
constexpr auto skip_before = directive_modifier<directives::preskip, directives::postskip, directives::eps>{};
template <unicode::ctype Property> struct ctype_combinator { void operator()(encoder& d) const { d.match_any(Property); } };

namespace language
{

using lug::grammar; using lug::rule; using lug::start;
using unicode::ctype; using unicode::ptype; using unicode::gctype; using unicode::sctype;
using unicode::blktype; using unicode::agetype; using unicode::eawtype;
using parser = lug::parser; using syntax = lug::syntax; using csyntax = lug::syntax const;
using syntax_position = lug::syntax_position; using syntax_range = lug::syntax_range;
using environment = lug::environment; template <class T> using variable = lug::variable<T>;
constexpr auto cased = directive_modifier<directives::none, directives::caseless, directives::eps>{};
constexpr auto caseless = directive_modifier<directives::caseless, directives::none, directives::eps>{};
constexpr auto lexeme = directive_modifier<directives::lexeme, directives::noskip, directives::eps>{};
constexpr auto noskip = directive_modifier<directives::lexeme | directives::noskip, directives::none, directives::eps>{};
constexpr auto skip = directive_modifier<directives::none, directives::lexeme | directives::noskip, directives::eps>{};
constexpr struct { void operator()(encoder&) const {} } nop = {};
constexpr struct { void operator()(encoder& d) const { d.match_eps(); } } eps = {};
constexpr struct { void operator()(encoder& d) const { d.encode(opcode::choice, 2).encode(opcode::match_any).encode(opcode::fail, immediate{1}); } } eoi = {};
constexpr struct { void operator()(encoder& d) const { d.encode(opcode::match_eol); } } eol = {};
constexpr struct { void operator()(encoder& d) const { d.encode(opcode::accept); } } cut = {};
constexpr ctype_combinator<ctype::alpha> alpha = {}; constexpr ctype_combinator<ctype::alnum> alnum = {}; constexpr ctype_combinator<ctype::lower> lower = {};
constexpr ctype_combinator<ctype::upper> upper = {}; constexpr ctype_combinator<ctype::digit> digit = {}; constexpr ctype_combinator<ctype::xdigit> xdigit = {};
constexpr ctype_combinator<ctype::space> space = {}; constexpr ctype_combinator<ctype::blank> blank = {}; constexpr ctype_combinator<ctype::punct> punct = {};
constexpr ctype_combinator<ctype::graph> graph = {}; constexpr ctype_combinator<ctype::print> print = {};

constexpr struct
{
	void operator()(encoder& d) const { d.match_any(); }
	template <class T, class = std::enable_if_t<unicode::is_property_enum_v<T>>>
	auto operator()(T p) const { return [p](encoder& d) { d.match_any(p); }; }
}
any = {};

constexpr struct
{
	template <class T, class = std::enable_if_t<unicode::is_property_enum_v<T>>>
	auto operator()(T p) const { return [p](encoder& d) { d.match_all(p); }; }
}
all = {};

constexpr struct
{
	template <class T, class = std::enable_if_t<unicode::is_property_enum_v<T>>>
	auto operator()(T p) const { return [p](encoder& d) { d.match_none(p); }; }
}
none = {};

constexpr struct
{
	auto operator()(std::string_view s) const { return basic_regular_expression{s}; }
	auto operator()(char const* s, std::size_t n) const { return basic_regular_expression{std::string_view{s, n}}; }
}
bre = {};

constexpr struct
{
	auto operator()(char32_t c) const
	{
		return [c](encoder& d) { d.match(utf8::encode_rune(c)); };
	}

	auto operator()(char32_t start, char32_t end) const
	{
		return [start, end](encoder& d) {
			d.match(unicode::sort_and_optimize(add_rune_range(unicode::rune_set{}, d.mode(), start, end)));
		};
	}
}
chr = {};

constexpr struct
{
	auto operator()(std::string_view s) const { return string_expression{s}; }
	auto operator()(char const* s, std::size_t n) const { return string_expression{std::string_view{s, n}}; }
}
str = {};

inline auto operator ""_cx(char32_t c) { return chr(c); }
inline auto operator ""_sx(char const* s, std::size_t n) { return string_expression{std::string_view{s, n}}; }
inline auto operator ""_rx(char const* s, std::size_t n) { return basic_regular_expression{std::string_view{s, n}}; }
inline auto operator ""_icx(char32_t c) { return caseless[chr(c)]; }
inline auto operator ""_isx(char const* s, std::size_t n) { return caseless[string_expression{std::string_view{s, n}}]; }
inline auto operator ""_irx(char const* s, std::size_t n) { return caseless[basic_regular_expression{std::string_view{s, n}}]; }
inline auto operator ""_scx(char32_t c) { return cased[chr(c)]; }
inline auto operator ""_ssx(char const* s, std::size_t n) { return cased[string_expression{std::string_view{s, n}}]; }
inline auto operator ""_srx(char const* s, std::size_t n) { return cased[basic_regular_expression{std::string_view{s, n}}]; }

struct implicit_space_rule
{
        std::function<void(encoder&)> save_;

	template <class E, class = std::enable_if_t<is_expression_v<E>>>
	implicit_space_rule(E const& e) : save_{grammar::implicit_space}
	{
		grammar::implicit_space = std::function<void(encoder&)>{make_expression(e)};
	}

        ~implicit_space_rule() 
        {
                grammar::implicit_space = std::move(save_);
        }
};

template <class E, class = std::enable_if_t<is_expression_v<E>>>
constexpr auto operator!(E const& e)
{
	return [x = matches_eps[e]](encoder& d) {
		d.encode(opcode::choice, 1 + d.evaluate_length(x)).evaluate(x).encode(opcode::fail, immediate{1});
	};
}

template <class E, class = std::enable_if_t<is_expression_v<E>>>
constexpr auto operator&(E const& e)
{
	return [x = matches_eps[e]](encoder& d) {
		d.encode(opcode::choice, 2 + d.evaluate_length(x)).evaluate(x).encode(opcode::commit_back, 1).encode(opcode::fail);
	};
}

template <class E, class = std::enable_if_t<is_expression_v<E>>>
constexpr auto operator*(E const& e)
{
	return [x = matches_eps[skip_after[e]]](encoder& d) {
		auto n = d.evaluate_length(x);
		d.encode(opcode::choice, 2 + n).evaluate(x).encode(opcode::commit_partial, -(2 + n));
	};
}

template <class E1, class E2, class = std::enable_if_t<is_expression_v<E1> && is_expression_v<E2>>>
constexpr auto operator|(E1 const& e1, E2 const& e2)
{
	return [x1 = relays_eps[e1], x2 = relays_eps[e2]](encoder& d) {
		d.encode(opcode::choice, 2 + d.evaluate_length(x1)).evaluate(x1).encode(opcode::commit, d.evaluate_length(x2)).evaluate(x2);
	};
}

template <class E1, class E2, class = std::enable_if_t<is_expression_v<E1> && is_expression_v<E2>>>
constexpr auto operator>(E1 const& e1, E2 const& e2)
{
	return [x1 = make_expression(e1), x2 = skip_before[e2]](encoder& d) { d.evaluate(x1).evaluate(x2); };
}

template <class E, class A, class = std::enable_if_t<is_expression_v<E>>>
constexpr auto operator<(E const& e, A a)
{
	if constexpr (std::is_invocable_v<A, environment&, syntax>) {
		return [e = make_expression(e), a = ::std::move(a)](encoder& d) {
			d.skip().encode(opcode::begin).evaluate(e).encode(opcode::end, syntactic_capture{a});
		};
	} else if constexpr (std::is_invocable_v<A, detail::dynamic_cast_if_base_of<environment&>, syntax>) {
		return e < [a = ::std::move(a)](environment& envr, csyntax& x) {
			a(detail::dynamic_cast_if_base_of<environment&>{envr}, x);
		};
	} else if constexpr (std::is_invocable_v<A, syntax>) {
		return e < [a = ::std::move(a)](environment&, csyntax& x) {
			a(x);
		};
	} else if constexpr (std::is_invocable_v<A, environment&>) {
		return [e = make_expression(e), a = ::std::move(a)](encoder& d) {
			d.evaluate(e).encode(opcode::action, semantic_action{a});
		};
	} else if constexpr (std::is_invocable_v<A, detail::dynamic_cast_if_base_of<environment&>>) {
		return e < [a = ::std::move(a)](environment& envr) {
			a(detail::dynamic_cast_if_base_of<environment&>{envr});
		};
	} else if constexpr (std::is_invocable_v<A> && std::is_same_v<void, std::invoke_result_t<A>>) {
		return [e = make_expression(e), a = ::std::move(a)](encoder& d) {
			d.evaluate(e).encode(opcode::action, [a](environment&) { a(); });
		};
	} else if constexpr (std::is_invocable_v<A>) {
		return [e = make_expression(e), a = ::std::move(a)](encoder& d) {
			d.evaluate(e).encode(opcode::action, [a](environment& envr) { envr.push_attribute(a()); });
		};
	}
}

template <class E, class = std::enable_if_t<is_expression_v<E>>>
constexpr auto operator+(E const& e)
{
	auto x = make_expression(e);
	return x > *x;
}

template <class E, class = std::enable_if_t<is_expression_v<E>>>
constexpr auto operator~(E const& e)
{
	return e | eps;
}

template <class E, class = std::enable_if_t<is_expression_v<E>>>
constexpr auto operator--(E const& e)
{
	return cut > e;
}

template <class E, class = std::enable_if_t<is_expression_v<E>>>
constexpr auto operator--(E const& e, int)
{
	return e > cut;
}

constexpr struct
{
	template <class T>
	struct capture_to
	{
		variable<T>& v;

		template <class E, class = std::enable_if_t<is_expression_v<E>>>
		constexpr auto operator[](E const& e) const
		{
			return e < [&vr = v](syntax x) { *vr = T(x.capture()); };
		}
	};

	template <class T>
	constexpr auto operator()(variable<T>& v) const
	{
		return capture_to<T>{v};
	}
}
capture = {};

template <class T, class E, class = std::enable_if_t<is_expression_v<E>>>
inline auto operator%(variable<T>& v, E const& e)
{
	return e < [&v](environment& s) { *v = s.pop_attribute<T>(); };
}

} // namespace language

grammar start(rule const& start_rule);

struct parser_registers
{
	std::size_t sr, mr, rc; std::ptrdiff_t pc; std::size_t fc;
	auto as_tuple() noexcept { return std::forward_as_tuple(sr, mr, rc, pc, fc); }
	auto as_tuple() const noexcept { return std::forward_as_tuple(sr, mr, rc, pc, fc); }
};

class parser
{
	enum class stack_frame_type : unsigned char { backtrack, call, capture, lrcall };
	enum class subject_location : std::size_t {};
	struct lrmemo { std::size_t srr, sra, prec; std::ptrdiff_t pcr, pca; std::size_t rcr; std::vector<semantic_response> responses; };
	static constexpr std::size_t lrfailcode = (std::numeric_limits<std::size_t>::max)();
	static constexpr unsigned short max_call_depth = (std::numeric_limits<unsigned short>::max)();
	static constexpr std::size_t max_size = (std::numeric_limits<std::size_t>::max)();
	lug::grammar const& grammar_;
	lug::environment& environment_;
	std::vector<std::function<bool(std::string&)>> sources_;
	std::string input_;
	std::unordered_map<std::size_t, std::string> casefolded_subjects_;
	syntax_position origin_{1, 1};
	std::vector<std::pair<std::size_t, syntax_position>> positions_;
	parser_registers registers_{0, 0, 0, 0, 0};
	bool parsing_{false}, reading_{false}, cut_deferred_{false};
	std::size_t cut_frame_{0};
	std::vector<stack_frame_type> stack_frames_;
	std::vector<std::tuple<std::size_t, std::size_t, std::ptrdiff_t>> backtrack_stack_; // sr, rc, pc
	std::vector<std::ptrdiff_t> call_stack_; // pc
	std::vector<subject_location> capture_stack_; // sr
	std::vector<lrmemo> lrmemo_stack_;
	std::vector<semantic_response> responses_;
	unsigned short prune_depth_{max_call_depth}, call_depth_{0};

	bool available(std::size_t sr, std::size_t sn);

	bool read_more();

	int casefold_compare(std::size_t sr, std::size_t sn, std::string_view str);

	template <class Compare>
	bool match_sequence(std::size_t& sr, std::string_view str, Compare&& comp)
	{
		if (auto sn = str.size(); !sn || (available(sr, sn) && comp(sr, sn, str))) {
			sr += sn;
			return true;
		}
		return false;
	}

	template <class Match>
	bool match_single(std::size_t& sr, Match&& match)
	{
		if (!available(sr, 1))
			return false;
		auto const curr = input_.cbegin() + sr, last = input_.cend();
		auto [next, rune] = utf8::decode_rune(curr, last);
		bool matched;
		if constexpr (std::is_invocable_v<Match, decltype(curr), decltype(last), decltype(next)&, char32_t>) {
			matched = match(curr, last, next, rune);
		} else if constexpr(std::is_invocable_v<Match, unicode::record const&>) {
			matched = match(unicode::query(rune));
		} else if constexpr(std::is_invocable_v<Match, char32_t>) {
			matched = match(rune);
		} else {
			matched = match();
			detail::ignore(rune);
		}
		if (matched)
			sr += std::distance(curr, next);
		return matched;
	}

        // template lug::parser::commit<opcode::commit>
        // template lug::parser::commit<opcode::commit_back>
        // template lug::parser::commit<opcode::commit_partial>
	template <opcode Opcode>
	bool commit(std::size_t& sr, std::size_t& rc, std::ptrdiff_t& pc, int off);

	void accept(std::size_t sr, std::size_t mr, std::size_t rc, std::ptrdiff_t pc);

	auto drain() -> decltype(std::declval<parser_registers>().as_tuple());

	void pop_responses_after(std::size_t n);

	auto drop_responses_after(std::size_t n) -> std::vector<semantic_response>;

	auto restore_responses_after(std::size_t n, std::vector<semantic_response> const& restore) -> size_t;

	auto push_response(std::size_t depth, std::size_t action_index, syntax_range range = {max_size, 0}) -> size_t;

	template <class Stack, class... Args>
	void pop_stack_frame(Stack& stack, Args&... args)
	{
		stack.pop_back(), stack_frames_.pop_back();
		cut_frame_ = (std::min)(cut_frame_, stack_frames_.size());
		if constexpr (std::is_same_v<typename Stack::value_type, subject_location> || std::is_same_v<typename Stack::value_type, lrmemo>)
			if (cut_deferred_ && capture_stack_.empty() && lrmemo_stack_.empty())
				accept(args...);
	}

public:
	parser(lug::grammar const& g, lug::environment& e) : grammar_{g}, environment_{e} {}
	lug::grammar const& grammar() const noexcept { return grammar_; }
	lug::environment& environment() const noexcept { return environment_; }
	std::string_view match() const noexcept { return {input_.data(), registers_.sr}; }
	std::string_view subject() const noexcept { return {input_.data() + registers_.sr, input_.size() - registers_.sr}; }
	std::size_t subject_index() const noexcept { return registers_.sr; }
	std::size_t max_subject_index() const noexcept { return registers_.mr; }
	syntax_position const& subject_position() { return position_at(registers_.sr); }
	syntax_position const& max_subject_position() { return position_at(registers_.mr); }
	parser_registers& registers() noexcept { return registers_; }
	parser_registers const& registers() const noexcept { return registers_; }
	bool available(std::size_t sn) { return available(registers_.sr, sn); }
	unsigned short call_depth() const noexcept { return call_depth_; }
	unsigned short prune_depth() const noexcept { return prune_depth_; }
	void escape() { prune_depth_ = call_depth_; }

	syntax_position const& position_at(std::size_t index);

	template <class InputIt, class = utf8::enable_if_char_input_iterator_t<InputIt>>
	parser& enqueue(InputIt first, InputIt last)
	{
		input_.insert(input_.end(), first, last);
		return *this;
	}

	template <class InputFunc, class = std::enable_if_t<std::is_invocable_r_v<bool, InputFunc, std::string&>>>
	parser& push_source(InputFunc&& func)
	{
		if (reading_)
			throw reenterant_read_error{};
		sources_.emplace_back(::std::forward<InputFunc>(func));
		return *this;
	}

	template <class InputIt, class = utf8::enable_if_char_input_iterator_t<InputIt>>
	bool parse(InputIt first, InputIt last)
	{
		return enqueue(first, last).parse();
	}

	template <class InputFunc, class = std::enable_if_t<std::is_invocable_r_v<bool, InputFunc, std::string&>>>
	bool parse(InputFunc&& func)
	{
		return push_source(::std::forward<InputFunc>(func)).parse();
	}

	bool parse();
};

template <class InputIt, class = utf8::enable_if_char_input_iterator_t<InputIt>>
inline bool parse(InputIt first, InputIt last, grammar const& grmr, environment& envr)
{
	return parser{grmr, envr}.enqueue(first, last).parse();
}

template <class InputIt, class = utf8::enable_if_char_input_iterator_t<InputIt>>
inline bool parse(InputIt first, InputIt last, grammar const& grmr)
{
	environment envr;
	return parse(first, last, grmr, envr);
}

inline bool parse(std::istream& input, grammar const& grmr, environment& envr)
{
	return parser{grmr, envr}.push_source([&input](std::string& line) {
		if (std::getline(input, line)) {
			line.push_back('\n');
			return true;
		}
		return false;
	}).parse();
}

inline bool parse(std::istream& input, grammar const& grmr)
{
	environment envr;
	return parse(input, grmr, envr);
}

inline bool parse(std::string_view sv, grammar const& grmr, environment& envr)
{
	return parse(sv.cbegin(), sv.cend(), grmr, envr);
}

inline bool parse(std::string_view sv, grammar const& grmr)
{
	return parse(sv.cbegin(), sv.cend(), grmr);
}

inline bool parse(grammar const& grmr, environment& envr)
{
	return parse(std::cin, grmr, envr);
}

inline bool parse(grammar const& grmr)
{
	return parse(std::cin, grmr);
}

inline std::string_view syntax::capture() const { return parser_.match().substr(range_.index, range_.size); }
inline syntax_position const& syntax::start() const { return parser_.position_at(range_.index); }
inline syntax_position const& syntax::end() const { return parser_.position_at(range_.index + range_.size); }

inline unsigned short environment::call_depth() const { return parser().call_depth(); }
inline unsigned short environment::prune_depth() const { return parser().prune_depth(); }
inline void environment::escape() { parser().escape(); }
inline std::string_view environment::match() const { return parser().match(); }
inline syntax_position const& environment::position_at(std::size_t index) { return parser().position_at(index); }

LUG_DIAGNOSTIC_PUSH_AND_IGNORE

LUG_DIAGNOSTIC_POP

} // namespace lug

extern template class std::vector<lug::instruction>;
extern template class std::vector<std::tuple<lug::rule const*, lug::program const*, std::ptrdiff_t, lug::directives>>;
extern template class std::vector<lug::instruction>;
extern template class std::vector<lug::unicode::rune_set>;
extern template class std::vector<lug::semantic_predicate>;
extern template class std::vector<lug::semantic_action>;
extern template class std::vector<lug::syntactic_capture>;
extern template class std::vector<lug::semantic_response>;
extern template class std::vector<enum lug::directives>;
