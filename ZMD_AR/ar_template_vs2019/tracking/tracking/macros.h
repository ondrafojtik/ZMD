#ifndef MACROS_H_
#define MACROS_H_

#define TYPE_REAL float

// A macro to disallow the copy constructor and operator= functions
// This should be used in the private: declarations for a class
#define DISALLOW_COPY_AND_ASSIGN( type_name ) \
  type_name( const type_name & ); \
  void operator=( const type_name & )

#define SAFE_DELETE( p ) { \
	if ( p != NULL ) \
				{ \
	delete p; \
	p = NULL; \
			} \
}

#define SAFE_DELETE_ARRAY( p ) { \
	if ( p != NULL ) \
				{ \
		delete [] p; \
		p = NULL; \
				} \
}

#define SQR( x ) ( ( x ) * ( x ) )
#define DEG2RAD( x ) ( ( x ) * M_PI / 180.0 )

#endif