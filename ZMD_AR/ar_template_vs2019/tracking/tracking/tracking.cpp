#include "stdafx.h"

int main( int argc, char * argv[] )
{
	printf( "ZMD, (c)2020 Tomas Fabian\n\n" );

	printf( "Using OpenCV %s\n", cv::getVersionString().c_str() );

	const char * locale = setlocale( LC_ALL, "" );
	std::locale lollocale( locale );
	setlocale( LC_ALL, locale );
	setlocale( LC_NUMERIC, "C" );
	
	MBAR mbar( "..//..//data//test03.png" );
	mbar.start();

	return 0;
}
