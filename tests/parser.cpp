// lug - Embedded DSL for PE grammar parser combinators in C++
// Copyright (c) 2017 Jesse W. Towner
// See LICENSE.md file for license details

#include <lug/lug.hpp>
#include <cassert>

using namespace std::string_view_literals;

constexpr auto text =
u8R"(The stranger officiates the meal.
She was too short to see over the fence.
This is the last random sentence I will be writing and I am going to stop mid-sent
It was getting dark, and we weren't there yet.
There were white out conditions in the town; subsequently, the roads were impassable.
I really want to go to work, but I am too sick to drive.
I am counting my calories, yet I really want dessert.
I checked to make sure that he was still alive.
I love eating toasted cheese and tuna sandwiches.
Everyone was busy, so I went to the movie alone.
He got a small piece of pie.
What was the person thinking when they discovered cow's milk was fine for human consumption... and why did they do it in the first place!?
Where do random thoughts come from?
If Purple People Eaters are real... where do they find purple people to eat?
He turned in the research paper on Friday; otherwise, he would have not passed the class.
How was the math test?
The mysterious diary records the voice.
She works two jobs to make ends meet; at least, that was her reason for not having time to join us.
I want more detailed information.
He told us a very exciting adventure story.)"sv;

void test_line_column_tracking()
{
	using namespace lug::language;

	std::array<syntax_position, 4> startpos, endpos;

	rule Word = lexeme[
			  str("officiates") < [&](csyntax& x) { startpos[0] = x.start(); endpos[0] = x.end(); }
			| str("Everyone") < [&](csyntax& x) { endpos[1] = x.end(); startpos[1] = x.start();  }
			| str("Friday") < [&](csyntax& x) { startpos[2] = x.start(); endpos[2] = x.end(); }
			| str("story") < [&](csyntax& x) { endpos[3] = x.end(); startpos[3] = x.start(); }
			| +alpha
		];

	grammar G = start(*(Word | punct) > eoi);

	lug::environment E;
	lug::parser p{G, E};

	bool match = p.parse(std::begin(text), std::end(text));
	assert(match);

	assert(startpos[0].line == 1 && startpos[0].column == 14);
	assert(startpos[1].line == 10 && startpos[1].column == 1);
	assert(startpos[2].line == 15 && startpos[2].column == 36);
	assert(startpos[3].line == 20 && startpos[3].column == 38);

	assert(endpos[0].line == 1 && endpos[0].column == 24);
	assert(endpos[1].line == 10 && endpos[1].column == 9);
	assert(endpos[2].line == 15 && endpos[2].column == 42);
	assert(endpos[3].line == 20 && endpos[3].column == 30);
}

int main()
{
	try {
		test_line_column_tracking();
	} catch (std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return -1;
	}
	return 0;
}