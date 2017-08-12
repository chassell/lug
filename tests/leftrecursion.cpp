// lug - Embedded DSL for PE grammar parser combinators in C++
// Copyright (c) 2017 Jesse W. Towner
// See LICENSE.md file for license details

#include <lug/lug.hpp>
#include <cassert>

void test_direct_left_recursion()
{
	using namespace lug::language;
	constexpr auto A = lug::any_terminal{};
	using C = lug::char_terminal;
	rule S = (S > C{'a'} | C{'a'});
	grammar G = start(S > !C{'a'});
	assert(!lug::parse("", G));
	assert(!lug::parse("b", G));
	assert(lug::parse("a", G));
	assert(lug::parse("aa", G));
	assert(lug::parse("aab", G));
	assert(lug::parse("aaa", G));
	assert(lug::parse("aaa2", G));
	assert(lug::parse("aaaa", G));
	assert(lug::parse("aaaak", G));
}

void test_indirect_left_recursion()
{
	using namespace lug::language;
	constexpr auto A = lug::any_terminal{};
	using C = lug::char_terminal;
	rule Q, S;
	Q = S > C{'a'};
	S = Q | C{'a'};
	grammar G = start(S > !C{'a'});
	assert(!lug::parse("", G));
	assert(!lug::parse("b", G));
	assert(lug::parse("a", G));
	assert(lug::parse("aa", G));
	assert(lug::parse("aab", G));
	assert(lug::parse("aaa", G));
	assert(lug::parse("aaa2", G));
	assert(lug::parse("aaaa", G));
	assert(lug::parse("aaaak", G));
}

void test_association_and_precedence()
{
	using namespace lug::language;
	constexpr auto A = lug::any_terminal{};
	using C = lug::char_terminal;
	std::string out;
	rule N	= C{'1'} | C{'2'} | C{'3'};
	rule E	= E[1] > C{'+'} > E[2] < [&out]() { out += '+'; }
			| E[2] > C{'*'} > E[3] < [&out]() { out += '*'; }
			| N < [&out](semantics&, syntax x) { out += x.capture; };
	grammar G = start(E > !A);
	assert(!lug::parse("", G));
	assert(!lug::parse("a", G));
	assert(!lug::parse("1+", G));
	out.clear();
	assert(lug::parse("1", G) && out == "1");
	out.clear();
	assert(lug::parse("1+2", G) && out == "12+");
	out.clear();
	assert(lug::parse("3*1", G) && out == "31*");
	out.clear();
	assert(lug::parse("1*2+3*2", G) && out == "12*32*+");
	out.clear();
	assert(lug::parse("2+2*3+1", G) && out == "223*+1+");
	out.clear();
	assert(lug::parse("2+2*3+1*2*3+1", G) && out == "223*+12*3*+1+");
}

int main()
{
	try {
		test_direct_left_recursion();
		test_indirect_left_recursion();
		test_association_and_precedence();
	} catch (std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return -1;
	}
	return 0;
}
