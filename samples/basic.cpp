// lug - Embedded DSL for PE grammar parser combinators in C++
// Copyright (c) 2017 Jesse W. Towner
// See LICENSE.md file for license details

// Derived from BASIC, Dartmouth College Computation Center, October 1st 1964
// http://www.bitsavers.org/pdf/dartmouth/BASIC_Oct64.pdf

#include <lug/lug.hpp>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <map>

#ifdef _MSC_VER
#include <io.h>
#define fileno _fileno
#define isatty _isatty
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#else
#include <unistd.h>
#endif

class basic_interpreter
{
public:
	basic_interpreter()
	{
		using namespace lug::language;
		rule Expr;

		implicit_space_rule SP = *"[ \t]"s;

		rule NL		= lexeme["\n"s | "\r\n" | "\r"];
		rule Func	= lexeme[capture(id_)["[A-Z]" > *"[A-Z0-9]"s]]  <[this]{ return *id_; };
		rule LineNo	= lexeme[capture(sv_)[+"[0-9]"s]]                     <[this]{ return std::stoi(std::string{*sv_}); };
		rule Real	= lexeme[capture(sv_)[+"[0-9]"s > ~("[.]"s > +"[0-9]"s)
					    > ~("[eE]"s > ~"[+-]"s > +"[0-9]"s)]]             <[this]{ return std::stod(std::string{*sv_}); };
		rule String	= lexeme["\"" > capture(sv_)[*"[^\"]"s] > "\""]       <[this]{ return *sv_; };
		rule Var	= lexeme[capture(id_)["[A-Z]" > ~"[0-9]"s]]        <[this]{ return *id_; };

		rule RelOp	= "="                                 <[]() -> RelOpFn { return [](double x, double y) { return x == y; }; }
					| ">="                                <[]() -> RelOpFn { return std::isgreaterequal; }
					| ">"                                 <[]() -> RelOpFn { return std::isgreater; }
					| "<="                                <[]() -> RelOpFn { return std::islessequal; }
					| "<>"                                <[]() -> RelOpFn { return [](double x, double y) { return x != y; }; }
					| "<"                                 <[]() -> RelOpFn { return std::isless; };

		rule Factor	= !(caseless["[A-Z][A-Z][A-Z]"] > "(")
					> (   id_%Var                                   <[this]{ return vars_[*id_]; }
					    | r1_%Real > ~(u8"[↑^]"s > r2_%Real         <[this]{ *r1_ = std::pow(*r1_, *r2_); }
					    )                                           <[this]{ return *r1_; }
					    | "(" > Expr > ")" )
					| caseless["SIN"] > "(" > r1_%Expr > ")"        <[this]{ return std::sin(*r1_); }
					| caseless["COS"] > "(" > r1_%Expr > ")"        <[this]{ return std::cos(*r1_); }
					| caseless["TAN"] > "(" > r1_%Expr > ")"        <[this]{ return std::tan(*r1_); }
					| caseless["ATN"] > "(" > r1_%Expr > ")"        <[this]{ return std::atan(*r1_); }
					| caseless["EXP"] > "(" > r1_%Expr > ")"        <[this]{ return std::exp(*r1_); }
					| caseless["ABS"] > "(" > r1_%Expr > ")"        <[this]{ return std::abs(*r1_); }
					| caseless["LOG"] > "(" > r1_%Expr > ")"        <[this]{ return std::log(*r1_); }
					| caseless["SQR"] > "(" > r1_%Expr > ")"        <[this]{ return std::sqrt(*r1_); }
					| caseless["INT"] > "(" > r1_%Expr > ")"        <[this]{ return std::trunc(*r1_); }
					| caseless["RND"] > "(" > r1_%Expr > ")"        <[this]{ return std::rand() / static_cast<double>(RAND_MAX); };

		rule Term	= r1_%Factor > *(
					      "*"s > r2_%Factor                         <[this]{ *r1_ *= *r2_; }
					    | "/"s > r2_%Factor                         <[this]{ *r1_ /= *r2_; }
					)                                               <[this]{ return *r1_; };

		     Expr	= (  ~ "+"s > r1_%Term
					     | "-"s > r1_%Term                          <[this]{ *r1_ = -*r1_; }
					) > *( "+"s > r2_%Term                          <[this]{ *r1_ += *r2_; }
					     | "-"s > r2_%Term                          <[this]{ *r1_ -= *r2_; }
					)                                               <[this]{ return *r1_; };

		rule ReadEl = id_%Var                                       <[this]{ read(*id_); };
		rule DataEl = r1_%Real                                      <[this]{ data_.push_back(*r1_); };
		rule InptEl	= id_%Var                                       <[this]{ std::cin >> vars_[*id_]; };
		rule PrntEl	= sv_%String                                    <[this]{ std::cout << *sv_; }
					| r1_%Expr                                      <[this]{ std::cout << *r1_; };

		rule Stmnt	= caseless["PRINT"] > ~PrntEl > *("," > PrntEl) <[this]{ std::cout << std::endl; }
					| caseless["IF"] > r1_%Expr
					    > rop_%RelOp > r2_%Expr                     <[this]{ if (!(*rop_)(*r1_, *r2_)) { semantics_.escape(); } }
					    > caseless[ "THEN" ] > Stmnt
					| caseless["FOR"] > id_%Var > "=" > r1_%Expr
					    > caseless["TO"] > r2_%Expr
					    > ( caseless["STEP"] > r3_%Expr
					      | eps < [this]{ *r3_ = 1.0; } )           <[this]{ for_to_step(*id_, *r1_, *r2_, *r3_); }
					| caseless["NEXT"] > id_%Var                    <[this]{ next(*id_); }
					| caseless["GOTO"] > no_%LineNo                 <[this]{ goto_line(*no_); }
					| caseless["READ"] > ReadEl > *("," > ReadEl)
					| caseless["RESTORE"]                           <[this]{ read_itr_ = data_.cbegin(); }
					| caseless["INPUT"] > InptEl > *("," > InptEl)
					| caseless["LET"] > id_%Var > "=" > r1_%Expr    <[this]{ vars_[*id_] = *r1_; }
					| caseless["GOSUB"] > no_%LineNo                <[this]{ gosub(*no_); }
					| caseless["RETURN"]                            <[this]{ retsub(); }
					| caseless["STOP"]                              <[this]{ haltline_ = line_; line_ = lines_.end(); }
					| caseless["END"]                               <[this]{ if (line_ == lines_.end()) std::exit(EXIT_SUCCESS); line_ = lines_.end(); }
					| (caseless["EXIT"] | caseless["QUIT"])         <[this]{ std::exit(EXIT_SUCCESS); }
					| caseless["REM"] > *(!NL > any);

		rule Cmnd	= caseless["CLEAR"]                             <[this]{ lines_.clear(); }
					| caseless["CONT"]                              <[this]{ cont(); }
					| caseless["LIST"]                              <[this]{ list(std::cout); }
					| caseless["LOAD"] > sv_%String                 <[this]{ load(*sv_); }
					| caseless["RUN"]                               <[this]{ line_ = lines_.begin(); read_itr_ = data_.cbegin(); }
					| caseless["SAVE"] > sv_%String                 <[this]{ save(*sv_); };

		rule Line	= Stmnt > NL
					| Cmnd > NL
					| no_%LineNo > capture(sv_)[*(!NL > any) > NL]  <[this]{ update_line(*no_, *sv_); }
					| NL
					| (*(!NL > any) > NL)                           <[this]{ print_error("ILLEGAL FORMULA"); }
					| !any                                          <[this]{ std::exit(EXIT_SUCCESS); };

		grammar_ = start(Line);
	}

	void repl()
	{
		lug::parser parser{grammar_, semantics_};
		parser.push_source([this](std::string& out) {
			if (line_ != lines_.end()) {
				lastline_ = line_++;
				out = lastline_->second;
			} else {
				if (stdin_tty_)
					std::cout << "> " << std::flush;
				if (!std::getline(std::cin, out))
					return false;
				out.push_back('\n');
			}
			return true;
		});
		while (parser.parse()) ;
	}

	void load(std::string_view name)
	{
		std::ifstream file{filename_with_ext(name), std::ifstream::in};
		if (file) {
			while (!file.bad() && !file.eof()) {
				int lineno;
				std::string line;
				if (file >> lineno && std::getline(file >> std::ws, line)) {
					update_line(lineno, line + "\n");
				} else {
					file.clear();
					file.ignore(std::numeric_limits<std::streamsize>::max(), file.widen('\n'));
				}
			}
		} else {
			print_error("FILE DOES NOT EXIST");
		}
	}

private:
	std::string filename_with_ext(std::string_view name)
	{
		std::string filename{name};
		if (filename.size() < 4 || strncasecmp(filename.data() + filename.size() - 4, ".BAS", 4) != 0)
			filename.append(".BAS");
		return filename;
	}

	void print_error(const char* message)
	{
		std::cerr << message << "\n";
		if (lastline_ != lines_.end())
			std::cerr << "LINE " << lastline_->first << ": " << lastline_->second << std::flush;
		line_ = lastline_ = lines_.end();
		stack_.clear(), for_stack_.clear();
	}

	void cont()
	{
		line_ = haltline_;
		haltline_ = lines_.end();
		if (line_ == haltline_)
			print_error("CAN'T CONTINUE");
	}

	void list(std::ostream& out)
	{
		for (const auto& [n, l] : lines_)
			out << n << "\t" << l;
		out << std::flush;
	}

	void save(std::string_view name)
	{
		std::ofstream file{filename_with_ext(name), std::ofstream::out};
		if (file)
			list(file);
		else
			print_error("UNABLE TO SAVE TO FILE");
	}

	void update_line(int n, std::string_view s)
	{
		haltline_ = lines_.end();
		if (s.empty() || s.front() < ' ')
			lines_.erase(n);
		else
			lines_[n] = s;
	}

	bool goto_line(int n)
	{
		if (lastline_ = line_, line_ = lines_.find(n); line_ == lines_.end()) {
			print_error("ILLEGAL LINE NUMBER");
			return false;
		}
		return true;
	}

	void gosub(int n)
	{
		lastline_ = line_;
		if (goto_line(n))
			stack_.push_back(lastline_);
	}

	void retsub()
	{
		if (!stack_.empty())
			line_ = stack_.back(), stack_.pop_back();
		else
			print_error("ILLEGAL RETURN");
	}

	void for_to_step(const std::string& id, double from, double to, double step)
	{
		if (lastline_ != lines_.end()) {
			double& v = vars_[id];
			v += step;
			if (for_stack_.empty() || id != for_stack_.back().first) {
				for_stack_.emplace_back(id, lastline_);
				v = from;
			}
			if ((step >= 0 && v <= to) || (step < 0 && v >= to))
				return;
			for_stack_.pop_back();
			for ( ; line_ != lines_.end(); ++line_) {
				if (auto& t = line_->second; t.compare(0, 4, "NEXT") == 0) {
					if (auto i = t.find_first_not_of(" \t", 4); i != std::string::npos && t.compare(i, id.size(), id) == 0) {
						lastline_ = line_++;
						return;
					}
				}
			}
		}
		print_error("FOR WITHOUT NEXT");
	}

	void next(const std::string& id)
	{
		if (lastline_ != lines_.end() && !for_stack_.empty() && for_stack_.back().first == id) {
			lastline_ = line_;
			line_ = for_stack_.back().second;
		} else {
			print_error("NOT MATCH WITH FOR");
		}
	}

	void read(const std::string& id)
	{
		if (read_itr_ != data_.cend())
			vars_[id] = *(read_itr_++);
		else
			print_error("NO DATA");
	}

	using RelOpFn = bool(*)(double, double);
	lug::grammar grammar_;
	lug::semantics semantics_;
	lug::variable<std::string> id_{semantics_};
	lug::variable<std::string_view> sv_{semantics_};
	lug::variable<double> r1_{semantics_}, r2_{semantics_}, r3_{semantics_};
	lug::variable<int> no_{semantics_};
	lug::variable<RelOpFn> rop_{semantics_};
	std::deque<double> data_;
	std::deque<double>::const_iterator read_itr_;
	std::unordered_map<std::string, double> vars_;
	std::map<int, std::string> lines_;
	std::map<int, std::string>::iterator line_{lines_.end()}, lastline_{lines_.end()}, haltline_{lines_.end()};
	std::vector<std::map<int, std::string>::iterator> stack_;
	std::vector<std::pair<std::string, std::map<int, std::string>::iterator>> for_stack_;
	const bool stdin_tty_{isatty(fileno(stdin)) != 0};
};

int main(int argc, char** argv)
{
	try {
		basic_interpreter interpreter;
		while (--argc > 1)
			interpreter.load(*++argv);
		interpreter.repl();
	} catch (std::exception& e) {
		std::cerr << "ERROR: " << e.what() << std::endl;
		return -1;
	}
	return 0;
}
