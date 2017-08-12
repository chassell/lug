// lug - Embedded DSL for PE grammar parser combinators in C++
// Copyright (c) 2017 Jesse W. Towner
// See LICENSE.md file for license details

#include <lug/lug.hpp>
#include <cstdlib>

namespace samples::calc
{
	using namespace lug::language;

	semantics Sema;
	variable<std::string_view> m{Sema};
	variable<double> e{Sema}, l{Sema}, n{Sema}, r{Sema}, s{Sema};
	variable<int> i{Sema};
	double variables[26];

	rule _		= *"[ \t]"s;

	rule EOL	= "\n"s | "\r\n" | "\r" | ";";

	rule ID		= m<< "[a-z]"s > _           <[]()->int{ return m->at(0) - 'a'; };

	rule NUMBER = m<< (~"[-+]"s > +"[0-9]"s
				> ~("[.]"s > +"[0-9]"s)) > _ <[]{ return std::stod(std::string{*m}); };

	extern rule Expr;

	rule Value	= n%NUMBER                   <[]{ return *n; }
				| i%ID > !( "=" > _ )        <[]{ return variables[*i]; }
				| "(" > _ > e%Expr > ")" > _ <[]{ return *e; };

	rule Prod	= l%Value > *(
				      "*" > _ > r%Value      <[]{ *l *= *r; }
				    | "/" > _ > r%Value      <[]{ *l /= *r; }
				)                            <[]{ return *l; };

	rule Sum	= l%Prod > *(
				      "+" > _ > r%Prod       <[]{ *l += *r; }
				    | "-" > _ > r%Prod       <[]{ *l -= *r; }
				)                            <[]{ return *l; };

	rule Expr	= i%ID > "=" > _ > s%Sum     <[]{ return variables[*i] = *s; }
				| s%Sum                      <[]{ return *s; };
	
	rule Stmt	= _ > (
				      "quit" > _             <[]{ std::exit(EXIT_SUCCESS); }
				    | e%Expr                 <[]{ std::cout << *e << std::endl; }
				) > EOL
				| *( !EOL > "." ) > EOL      <[]{ std::cerr << "syntax error" << std::endl; };

	grammar Grammar = start(Stmt);
}

int main()
{
	try {
		while(lug::parse(samples::calc::Grammar, samples::calc::Sema));
	} catch (std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return -1;
	}
	return 0;
}
