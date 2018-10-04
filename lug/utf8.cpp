// lug - Embedded DSL for PE grammar parser combinators in C++
// Copyright (c) 2017 Jesse W. Towner
// See LICENSE.md file for license details

// Based on the Flexible and Economical UTF-8 Decoder by Bjoern Hoehrmann
// Copyright (c) 2008-2010 Bjoern Hoehrmann <bjoern@hoehrmann.de>
// See LICENSE.md file or http://bjoern.hoehrmann.de/utf-8/decoder/dfa/
// for more details.

#include "lug/utf8.hpp"

unsigned int lug::utf8::decode_rune_octet(char32_t& rune, char octet, unsigned int state)
{
	static constexpr std::array<unsigned char, 256> dfa_class_table
	{
		 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
		 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
		 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
		 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
		 8, 8, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		10, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 3, 3,
		11, 6, 6, 6, 5, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8
	};

	static constexpr std::array<unsigned char, 108> dfa_transition_table
	{
		 0,12,24,36,60,96,84,12,12,12,48,72,12,12,12,12,
		12,12,12,12,12,12,12,12,12, 0,12,12,12,12,12, 0,
		12, 0,12,12,12,24,12,12,12,12,12,24,12,24,12,12,
		12,12,12,12,12,12,12,24,12,12,12,12,12,24,12,12,
		12,12,12,12,12,24,12,12,12,12,12,12,12,12,12,36,
		12,36,12,12,12,36,12,12,12,12,12,36,12,36,12,12,
		12,36,12,12,12,12,12,12,12,12,12,12
	};

	unsigned int const symbol = static_cast<unsigned char>(octet);
	unsigned int const dfa_class = dfa_class_table[symbol];
	rune = state == decode_accept ? symbol & (0xff >> dfa_class) : (symbol & 0x3f) | (rune << 6);
	return dfa_transition_table[state + dfa_class];
}

std::string lug::utf8::encode_rune(char32_t rune)
{
	std::string result;
	encode_rune(std::back_inserter(result), rune);
	return result;
}

std::string lug::utf8::tocasefold(std::string_view src)
{
	std::string result;
	result.reserve(src.size());
	tocasefold(std::begin(src), std::end(src), std::back_inserter(result));
	return result;
}

std::string lug::utf8::tolower(std::string_view src)
{
	std::string result;
	result.reserve(src.size());
	tolower(std::begin(src), std::end(src), std::back_inserter(result));
	return result;
}

std::string lug::utf8::toupper(std::string_view src)
{
	std::string result;
	result.reserve(src.size());
	toupper(std::begin(src), std::end(src), std::back_inserter(result));
	return result;
}
