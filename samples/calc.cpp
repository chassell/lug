// lug - Embedded DSL for PE grammar parser combinators in C++
// Copyright (c) 2017 Jesse W. Towner
// See LICENSE.md file for license details

#define DEBUG { std::cerr << "line=" << __LINE__ << std::endl; }

#include "lug/lug.hpp"
#include <cstdlib>
#include <iostream>

namespace samples::calc
{
	using namespace lug::language;

	environment Env;
	variable<std::string_view> m{Env};
	variable<double> e{Env}, l{Env}, n{Env}, r{Env}, s{Env};
	variable<int> i{Env};
	double v[26];

	extern rule Expr;

	implicit_space_rule BLANK = lexeme[ *"[ \t]"_rx ];

	rule EOL	= lexeme[ "[\n\r;]"_rx ];
	rule ID		= lexeme[ capture(m)[ "[a-z]"_rx ] <[]() -> int { DEBUG; return m->at(0) - 'a'; } ];
	rule NUMBER = lexeme[ capture(m)[ ~"[-+]"_rx > +"[0-9]"_rx > ~("."_sx > +"[0-9]"_rx) ]
				    <[]{ DEBUG; return std::stod(std::string{*m}); } ];

	rule Value	= n%NUMBER               <[]{ DEBUG; return *n; }
				| i%ID > !"="_sx         <[]{ DEBUG; return v[*i]; }
				| "(" > e%Expr > ")"     <[]{ DEBUG; return *e; };
	rule Prod	= l%Value > *(
				      "*" > r%Value      <[]{ DEBUG; *l *= *r; }
				    | "/" > r%Value      <[]{ DEBUG; *l /= *r; }
				)                        <[]{ DEBUG; return *l; };
	rule Sum	= l%Prod > *(
				      "+" > r%Prod       <[]{ DEBUG; *l += *r; }
				    | "-" > r%Prod       <[]{ DEBUG; *l -= *r; }
				)                        <[]{ DEBUG; return *l; };
	rule Expr	= i%ID > "=" > s%Sum     <[]{ DEBUG; return v[*i] = *s; }
				| s%Sum                  <[]{ DEBUG; return *s; };
	rule Stmt	= (   "quit"_isx         <[]{ DEBUG; std::cerr.flush(); std::cout.flush(); std::exit(EXIT_SUCCESS); }
                            | e%Expr             <[]{ DEBUG; std::cerr << *e << std::endl; }
                          ) > EOL
			  | *( !EOL > any ) > EOL  <[]{ DEBUG; std::cerr << "syntax error" << std::endl; };

	grammar Grammar = start(Stmt);
}

int main()
{
    // lug::parse("quit\n",samples::calc::Grammar, samples::calc::Env);
	try {
		while (lug::parse(samples::calc::Grammar, samples::calc::Env)) ;
	} catch (std::exception const& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return -1;
	}
	return 0;
}
