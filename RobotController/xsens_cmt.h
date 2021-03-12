// Using the Xsens CMT DLL. Do NOT change this file.

/*! \file
	
	\brief CMT library main header file.
	
	This file is intended for people that want to use the Xsens CMT library.
	This file was automatically generated, so do not judge it based on its layout or
	redundant definitions.

	\section FileCopyright Copyright Notice 
	Copyright (C) Xsens Technologies B.V., 2006.  All rights reserved.

	This source code is intended for use only by Xsens Technologies BV and
	those that have explicit written permission to use it from
	Xsens Technologies BV.

	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
	PARTICULAR PURPOSE.
*/

#ifndef CMTDLL_PRE_H
#define CMTDLL_PRE_H

#define CMT_DLL_IMPORT

#ifdef _WIN32
#	ifdef __cplusplus
#		define CMT_DLL_API extern "C" __declspec(dllimport)
#	else
#		define CMT_DLL_API __declspec(dllimport)
#	endif
#else
#	ifdef HAVE_GCCVISIBILITYPATCH
#		define CMT_DLL_API extern "C" __attribute__ ((visibility("default")))
#	else
#		define CMT_DLL_API
#	endif
#endif

#endif	// CMTDLL_PRE_H

#ifndef XSENS_MONOLITHIC
#	define XSENS_MONOLITHIC
#endif
/*  A portable stdint.h
 ****************************************************************************
 *  BSD License:
 ****************************************************************************
 *
 *  Copyright (c) 2005-2007 Paul Hsieh
 *  All rights reserved.
 *  
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *  
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. The name of the author may not be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 *  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************
 *
 *  Version 0.1.11
 *
 *  The ANSI C standard committee, for the C99 standard, specified the
 *  inclusion of a new standard include file called stdint.h.  This is
 *  a very useful and long desired include file which contains several
 *  very precise definitions for integer scalar types that is
 *  critically important for making portable several classes of
 *  applications including cryptography, hashing, variable length
 *  integer libraries and so on.  But for most developers its likely
 *  useful just for programming sanity.
 *
 *  The problem is that most compiler vendors have decided not to
 *  implement the C99 standard, and the next C++ language standard
 *  (which has a lot more mindshare these days) will be a long time in
 *  coming and its unknown whether or not it will include stdint.h or
 *  how much adoption it will have.  Either way, it will be a long time
 *  before all compilers come with a stdint.h and it also does nothing
 *  for the extremely large number of compilers available today which
 *  do not include this file, or anything comparable to it.
 *
 *  So that's what this file is all about.  Its an attempt to build a
 *  single universal include file that works on as many platforms as
 *  possible to deliver what stdint.h is supposed to.  A few things
 *  that should be noted about this file:
 *
 *    1) It is not guaranteed to be portable and/or present an identical
 *       interface on all platforms.  The extreme variability of the
 *       ANSI C standard makes this an impossibility right from the
 *       very get go. Its really only meant to be useful for the vast
 *       majority of platforms that possess the capability of
 *       implementing usefully and precisely defined, standard sized
 *       integer scalars.  Systems which are not intrinsically 2s
 *       complement may produce invalid constants.
 *
 *    2) There is an unavoidable use of non-reserved symbols.
 *
 *    3) Other standard include files are invoked.
 *
 *    4) This file may come in conflict with future platforms that do
 *       include stdint.h.  The hope is that one or the other can be
 *       used with no real difference.
 *
 *    5) In the current verison, if your platform can't represent
 *       int32_t, int16_t and int8_t, it just dumps out with a compiler
 *       error.
 *
 *    6) 64 bit integers may or may not be defined.  Test for their
 *       presence with the test: #ifdef INT64_MAX or #ifdef UINT64_MAX.
 *       Note that this is different from the C99 specification which
 *       requires the existence of 64 bit support in the compiler.  If
 *       this is not defined for your platform, yet it is capable of
 *       dealing with 64 bits then it is because this file has not yet
 *       been extended to cover all of your system's capabilities.
 *
 *    7) (u)intptr_t may or may not be defined.  Test for its presence
 *       with the test: #ifdef PTRDIFF_MAX.  If this is not defined
 *       for your platform, then it is because this file has not yet
 *       been extended to cover all of your system's capabilities, not
 *       because its optional.
 *
 *    8) The following might not been defined even if your platform is
 *       capable of defining it:
 *
 *       WCHAR_MIN
 *       WCHAR_MAX
 *       (u)int64_t
 *       PTRDIFF_MIN
 *       PTRDIFF_MAX
 *       (u)intptr_t
 *
 *    9) The following have not been defined:
 *
 *       WINT_MIN
 *       WINT_MAX
 *
 *   10) The criteria for defining (u)int_least(*)_t isn't clear,
 *       except for systems which don't have a type that precisely
 *       defined 8, 16, or 32 bit types (which this include file does
 *       not support anyways). Default definitions have been given.
 *
 *   11) The criteria for defining (u)int_fast(*)_t isn't something I
 *       would trust to any particular compiler vendor or the ANSI C
 *       committee.  It is well known that "compatible systems" are
 *       commonly created that have very different performance
 *       characteristics from the systems they are compatible with,
 *       especially those whose vendors make both the compiler and the
 *       system.  Default definitions have been given, but its strongly
 *       recommended that users never use these definitions for any
 *       reason (they do *NOT* deliver any serious guarantee of
 *       improved performance -- not in this file, nor any vendor's
 *       stdint.h).
 *
 *   12) The following macros:
 *
 *       PRINTF_INTMAX_MODIFIER
 *       PRINTF_INT64_MODIFIER
 *       PRINTF_INT32_MODIFIER
 *       PRINTF_INT16_MODIFIER
 *       PRINTF_LEAST64_MODIFIER
 *       PRINTF_LEAST32_MODIFIER
 *       PRINTF_LEAST16_MODIFIER
 *       PRINTF_INTPTR_MODIFIER
 *
 *       are strings which have been defined as the modifiers required
 *       for the "d", "u" and "x" printf formats to correctly output
 *       (u)intmax_t, (u)int64_t, (u)int32_t, (u)int16_t, (u)least64_t,
 *       (u)least32_t, (u)least16_t and (u)intptr_t types respectively.
 *       PRINTF_INTPTR_MODIFIER is not defined for some systems which
 *       provide their own stdint.h.  PRINTF_INT64_MODIFIER is not
 *       defined if INT64_MAX is not defined.  These are an extension
 *       beyond what C99 specifies must be in stdint.h.
 *
 *       In addition, the following macros are defined:
 *
 *       PRINTF_INTMAX_HEX_WIDTH
 *       PRINTF_INT64_HEX_WIDTH
 *       PRINTF_INT32_HEX_WIDTH
 *       PRINTF_INT16_HEX_WIDTH
 *       PRINTF_INT8_HEX_WIDTH
 *       PRINTF_INTMAX_DEC_WIDTH
 *       PRINTF_INT64_DEC_WIDTH
 *       PRINTF_INT32_DEC_WIDTH
 *       PRINTF_INT16_DEC_WIDTH
 *       PRINTF_INT8_DEC_WIDTH
 *
 *       Which specifies the maximum number of characters required to
 *       print the number of that type in either hexadecimal or decimal.
 *       These are an extension beyond what C99 specifies must be in
 *       stdint.h.
 *
 *  Compilers tested (all with 0 warnings at their highest respective
 *  settings): Borland Turbo C 2.0, WATCOM C/C++ 11.0 (16 bits and 32
 *  bits), Microsoft Visual C++ 6.0 (32 bit), Microsoft Visual Studio
 *  .net (VC7), Intel C++ 4.0, GNU gcc v3.3.3
 *
 *  This file should be considered a work in progress.  Suggestions for
 *  improvements, especially those which increase coverage are strongly
 *  encouraged.
 *
 *  Acknowledgements
 *
 *  The following people have made significant contributions to the
 *  development and testing of this file:
 *
 *  Chris Howie
 *  John Steele Scott
 *  Dave Thorup
 *
 */

#include <stddef.h>
#include <limits.h>

/*
 *  For gcc with _STDINT_H, fill in the PRINTF_INT*_MODIFIER macros, and
 *  do nothing else.  On the Mac OS X version of gcc this is _STDINT_H_.
 */

#if (defined(__STDC__) && __STDC__ && __STDC_VERSION__ >= 199901L) || (defined (__WATCOMC__) && __WATCOMC__ >= 1250) || (defined(__GNUC__) && __GNUC__ > 2) || (defined(_MSC_VER) && _MSC_VER >= 1600)
#include <stdint.h>
#endif

#if  (defined (_STDINT_H_INCLUDED) || defined (_STDINT) || defined (_STDINT_H) || defined (_STDINT_H_)) && !defined (_PSTDINT_H_INCLUDED)
#define _PSTDINT_H_INCLUDED
# ifndef PRINTF_INT64_MODIFIER
#  if __WORDSIZE == 64
#   define PRINTF_INT64_MODIFIER "l"
#  else
#   define PRINTF_INT64_MODIFIER "ll"
#  endif
# endif
# ifndef PRINTF_INT32_MODIFIER
#  if (LONG_MAX == INT32_MAX) && !defined(__GNUC__)
#   define PRINTF_INT32_MODIFIER "l"
#  else
#   define PRINTF_INT32_MODIFIER ""
#  endif
# endif
# ifndef PRINTF_INT16_MODIFIER
#  define PRINTF_INT16_MODIFIER "h"
# endif
# ifndef PRINTF_INTMAX_MODIFIER
#  define PRINTF_INTMAX_MODIFIER PRINTF_INT64_MODIFIER
# endif
# ifndef PRINTF_INT64_HEX_WIDTH
#  define PRINTF_INT64_HEX_WIDTH "16"
# endif
# ifndef PRINTF_INT32_HEX_WIDTH
#  define PRINTF_INT32_HEX_WIDTH "8"
# endif
# ifndef PRINTF_INT16_HEX_WIDTH
#  define PRINTF_INT16_HEX_WIDTH "4"
# endif
# ifndef PRINTF_INT8_HEX_WIDTH
#  define PRINTF_INT8_HEX_WIDTH "2"
# endif
# ifndef PRINTF_INT64_DEC_WIDTH
#  define PRINTF_INT64_DEC_WIDTH "20"
# endif
# ifndef PRINTF_INT32_DEC_WIDTH
#  define PRINTF_INT32_DEC_WIDTH "10"
# endif
# ifndef PRINTF_INT16_DEC_WIDTH
#  define PRINTF_INT16_DEC_WIDTH "5"
# endif
# ifndef PRINTF_INT8_DEC_WIDTH
#  define PRINTF_INT8_DEC_WIDTH "3"
# endif
# ifndef PRINTF_INTMAX_HEX_WIDTH
#  define PRINTF_INTMAX_HEX_WIDTH PRINTF_INT64_HEX_WIDTH
# endif
# ifndef PRINTF_INTMAX_DEC_WIDTH
#  define PRINTF_INTMAX_DEC_WIDTH PRINTF_INT64_DEC_WIDTH
# endif

/*
 *  Something really weird is going on with Open Watcom.  Just pull some of
 *  these duplicated definitions from Open Watcom's stdint.h file for now.
 */

# if defined (__WATCOMC__) && __WATCOMC__ >= 1250
#  if !defined (INT64_C)
#   define INT64_C(x)   (x + (INT64_MAX - INT64_MAX))
#  endif
#  if !defined (UINT64_C)
#   define UINT64_C(x)  (x + (UINT64_MAX - UINT64_MAX))
#  endif
#  if !defined (INT32_C)
#   define INT32_C(x)   (x + (INT32_MAX - INT32_MAX))
#  endif
#  if !defined (UINT32_C)
#   define UINT32_C(x)  (x + (UINT32_MAX - UINT32_MAX))
#  endif
#  if !defined (INT16_C)
#   define INT16_C(x)   (x)
#  endif
#  if !defined (UINT16_C)
#   define UINT16_C(x)  (x)
#  endif
#  if !defined (INT8_C)
#   define INT8_C(x)   (x)
#  endif
#  if !defined (UINT8_C)
#   define UINT8_C(x)  (x)
#  endif
#  if !defined (UINT64_MAX)
#   define UINT64_MAX  18446744073709551615ULL
#  endif
#  if !defined (INT64_MAX)
#   define INT64_MAX  9223372036854775807LL
#  endif
#  if !defined (UINT32_MAX)
#   define UINT32_MAX  4294967295UL
#  endif
#  if !defined (INT32_MAX)
#   define INT32_MAX  2147483647L
#  endif
#  if !defined (INTMAX_MAX)
#   define INTMAX_MAX INT64_MAX
#  endif
#  if !defined (INTMAX_MIN)
#   define INTMAX_MIN INT64_MIN
#  endif
# endif
#endif

#ifndef _PSTDINT_H_INCLUDED
#define _PSTDINT_H_INCLUDED

#ifndef SIZE_MAX
# define SIZE_MAX (~(size_t)0)
#endif

/*
 *  Deduce the type assignments from limits.h under the assumption that
 *  integer sizes in bits are powers of 2, and follow the ANSI
 *  definitions.
 */

#ifndef UINT8_MAX
# define UINT8_MAX 0xff
#endif
#ifndef uint8_t
# if (UCHAR_MAX == UINT8_MAX) || defined (S_SPLINT_S)
    typedef unsigned char uint8_t;
#   define UINT8_C(v) ((uint8_t) v)
# else
#   error "Platform not supported"
# endif
#endif

#ifndef INT8_MAX
# define INT8_MAX 0x7f
#endif
#ifndef INT8_MIN
# define INT8_MIN INT8_C(0x80)
#endif
#ifndef int8_t
# if (SCHAR_MAX == INT8_MAX) || defined (S_SPLINT_S)
    typedef signed char int8_t;
#   define INT8_C(v) ((int8_t) v)
# else
#   error "Platform not supported"
# endif
#endif

#ifndef UINT16_MAX
# define UINT16_MAX 0xffff
#endif
#ifndef uint16_t
#if (UINT_MAX == UINT16_MAX) || defined (S_SPLINT_S)
  typedef unsigned int uint16_t;
# ifndef PRINTF_INT16_MODIFIER
#  define PRINTF_INT16_MODIFIER ""
# endif
# define UINT16_C(v) ((uint16_t) (v))
#elif (USHRT_MAX == UINT16_MAX)
  typedef unsigned short uint16_t;
# define UINT16_C(v) ((uint16_t) (v))
# ifndef PRINTF_INT16_MODIFIER
#  define PRINTF_INT16_MODIFIER "h"
# endif
#else
#error "Platform not supported"
#endif
#endif

#ifndef INT16_MAX
# define INT16_MAX 0x7fff
#endif
#ifndef INT16_MIN
# define INT16_MIN INT16_C(0x8000)
#endif
#ifndef int16_t
#if (INT_MAX == INT16_MAX) || defined (S_SPLINT_S)
  typedef signed int int16_t;
# define INT16_C(v) ((int16_t) (v))
# ifndef PRINTF_INT16_MODIFIER
#  define PRINTF_INT16_MODIFIER ""
# endif
#elif (SHRT_MAX == INT16_MAX)
  typedef signed short int16_t;
# define INT16_C(v) ((int16_t) (v))
# ifndef PRINTF_INT16_MODIFIER
#  define PRINTF_INT16_MODIFIER "h"
# endif
#else
#error "Platform not supported"
#endif
#endif

#ifndef UINT32_MAX
# define UINT32_MAX (0xffffffffUL)
#endif
#ifndef uint32_t
#if (UINT_MAX == UINT32_MAX) && !defined (S_SPLINT_S)
typedef unsigned int uint32_t;
# ifndef PRINTF_INT32_MODIFIER
#  define PRINTF_INT32_MODIFIER ""
# endif
# define UINT32_C(v) v ## U
#elif (ULONG_MAX == UINT32_MAX) || defined (S_SPLINT_S)
  typedef unsigned long uint32_t;
# define UINT32_C(v) v ## UL
# ifndef PRINTF_INT32_MODIFIER
#  define PRINTF_INT32_MODIFIER "l"
# endif
#elif (USHRT_MAX == UINT32_MAX)
  typedef unsigned short uint32_t;
# define UINT32_C(v) ((unsigned short) (v))
# ifndef PRINTF_INT32_MODIFIER
#  define PRINTF_INT32_MODIFIER ""
# endif
#else
#error "Platform not supported"
#endif
#endif

#ifndef INT32_MAX
# define INT32_MAX (0x7fffffffL)
#endif
#ifndef INT32_MIN
# define INT32_MIN INT32_C(0x80000000)
#endif
#ifndef int32_t
#if (INT_MAX == INT32_MAX) && defined (__GNUC__) && (__GNUC__ > 3)
  typedef signed int int32_t;
# define INT32_C(v) v
# ifndef PRINTF_INT32_MODIFIER
#  define PRINTF_INT32_MODIFIER ""
# endif
#elif (LONG_MAX == INT32_MAX) || defined (S_SPLINT_S)
  typedef signed long int32_t;
# define INT32_C(v) v ## L
# ifndef PRINTF_INT32_MODIFIER
#  define PRINTF_INT32_MODIFIER "l"
# endif
#elif (INT_MAX == INT32_MAX)
  typedef signed int int32_t;
# define INT32_C(v) v
# ifndef PRINTF_INT32_MODIFIER
#  define PRINTF_INT32_MODIFIER ""
# endif
#elif (SHRT_MAX == INT32_MAX)
  typedef signed short int32_t;
# define INT32_C(v) ((short) (v))
# ifndef PRINTF_INT32_MODIFIER
#  define PRINTF_INT32_MODIFIER ""
# endif
#else
#error "Platform not supported"
#endif
#endif

/*
 *  The macro stdint_int64_defined is temporarily used to record
 *  whether or not 64 integer support is available.  It must be
 *  defined for any 64 integer extensions for new platforms that are
 *  added.
 */

#undef stdint_int64_defined
#if (defined(__STDC__) && defined(__STDC_VERSION__)) || defined (S_SPLINT_S)
# if (__STDC__ && __STDC_VERSION >= 199901L) || defined (S_SPLINT_S)
#  define stdint_int64_defined
   typedef long long int64_t;
   typedef unsigned long long uint64_t;
#  define UINT64_C(v) v ## ULL
#  define  INT64_C(v) v ## LL
#  ifndef PRINTF_INT64_MODIFIER
#   define PRINTF_INT64_MODIFIER "ll"
#  endif
# endif
#endif

#if !defined (stdint_int64_defined)
# if defined(__GNUC__) && __WORDSIZE == 64
#  define stdint_int64_defined
   __extension__ typedef long int64_t;
   __extension__ typedef unsigned long uint64_t;
#  define UINT64_C(v) v ## UL
#  define  INT64_C(v) v ## L
#  ifndef PRINTF_INT64_MODIFIER
#   define PRINTF_INT64_MODIFIER "l"
#  endif
#elif defined(__GNUC__)
#  define stdint_int64_defined
   __extension__ typedef long long int64_t;
   __extension__ typedef unsigned long long uint64_t;
#  define UINT64_C(v) v ## ULL
#  define  INT64_C(v) v ## LL
#  ifndef PRINTF_INT64_MODIFIER
#   define PRINTF_INT64_MODIFIER "ll"
#  endif
# elif defined(__MWERKS__) || defined (__SUNPRO_C) || defined (__SUNPRO_CC) || defined (__APPLE_CC__) || defined (_LONG_LONG) || defined (_CRAYC) || defined (S_SPLINT_S)
#  define stdint_int64_defined
   typedef long long int64_t;
   typedef unsigned long long uint64_t;
#  define UINT64_C(v) v ## ULL
#  define  INT64_C(v) v ## LL
#  ifndef PRINTF_INT64_MODIFIER
#   define PRINTF_INT64_MODIFIER "ll"
#  endif
# elif (defined(__WATCOMC__) && defined(__WATCOM_INT64__)) || (defined(_MSC_VER) && _INTEGRAL_MAX_BITS >= 64) || (defined (__BORLANDC__) && __BORLANDC__ > 0x460) || defined (__alpha) || defined (__DECC)
#  define stdint_int64_defined
   typedef __int64 int64_t;
   typedef unsigned __int64 uint64_t;
#  define UINT64_C(v) v ## UI64
#  define  INT64_C(v) v ## I64
#  ifndef PRINTF_INT64_MODIFIER
#   define PRINTF_INT64_MODIFIER "I64"
#  endif
# endif
#endif

#if !defined (LONG_LONG_MAX) && defined (INT64_C)
# define LONG_LONG_MAX INT64_C (9223372036854775807)
#endif
#ifndef ULONG_LONG_MAX
# define ULONG_LONG_MAX UINT64_C (18446744073709551615)
#endif

#if !defined (INT64_MAX) && defined (INT64_C)
# define INT64_MAX INT64_C (9223372036854775807)
#endif
#if !defined (INT64_MIN) && defined (INT64_C)
# define INT64_MIN INT64_C (-9223372036854775808)
#endif
#if !defined (UINT64_MAX) && defined (INT64_C)
# define UINT64_MAX UINT64_C (18446744073709551615)
#endif

/*
 *  Width of hexadecimal for number field.
 */

#ifndef PRINTF_INT64_HEX_WIDTH
# define PRINTF_INT64_HEX_WIDTH "16"
#endif
#ifndef PRINTF_INT32_HEX_WIDTH
# define PRINTF_INT32_HEX_WIDTH "8"
#endif
#ifndef PRINTF_INT16_HEX_WIDTH
# define PRINTF_INT16_HEX_WIDTH "4"
#endif
#ifndef PRINTF_INT8_HEX_WIDTH
# define PRINTF_INT8_HEX_WIDTH "2"
#endif

#ifndef PRINTF_INT64_DEC_WIDTH
# define PRINTF_INT64_DEC_WIDTH "20"
#endif
#ifndef PRINTF_INT32_DEC_WIDTH
# define PRINTF_INT32_DEC_WIDTH "10"
#endif
#ifndef PRINTF_INT16_DEC_WIDTH
# define PRINTF_INT16_DEC_WIDTH "5"
#endif
#ifndef PRINTF_INT8_DEC_WIDTH
# define PRINTF_INT8_DEC_WIDTH "3"
#endif

/*
 *  Ok, lets not worry about 128 bit integers for now.  Moore's law says
 *  we don't need to worry about that until about 2040 at which point
 *  we'll have bigger things to worry about.
 */

#ifdef stdint_int64_defined
  typedef int64_t intmax_t;
  typedef uint64_t uintmax_t;
# define  INTMAX_MAX   INT64_MAX
# define  INTMAX_MIN   INT64_MIN
# define UINTMAX_MAX  UINT64_MAX
# define UINTMAX_C(v) UINT64_C(v)
# define  INTMAX_C(v)  INT64_C(v)
# ifndef PRINTF_INTMAX_MODIFIER
#   define PRINTF_INTMAX_MODIFIER PRINTF_INT64_MODIFIER
# endif
# ifndef PRINTF_INTMAX_HEX_WIDTH
#  define PRINTF_INTMAX_HEX_WIDTH PRINTF_INT64_HEX_WIDTH
# endif
# ifndef PRINTF_INTMAX_DEC_WIDTH
#  define PRINTF_INTMAX_DEC_WIDTH PRINTF_INT64_DEC_WIDTH
# endif
#else
  typedef int32_t intmax_t;
  typedef uint32_t uintmax_t;
# define  INTMAX_MAX   INT32_MAX
# define UINTMAX_MAX  UINT32_MAX
# define UINTMAX_C(v) UINT32_C(v)
# define  INTMAX_C(v)  INT32_C(v)
# ifndef PRINTF_INTMAX_MODIFIER
#   define PRINTF_INTMAX_MODIFIER PRINTF_INT32_MODIFIER
# endif
# ifndef PRINTF_INTMAX_HEX_WIDTH
#  define PRINTF_INTMAX_HEX_WIDTH PRINTF_INT32_HEX_WIDTH
# endif
# ifndef PRINTF_INTMAX_DEC_WIDTH
#  define PRINTF_INTMAX_DEC_WIDTH PRINTF_INT32_DEC_WIDTH
# endif
#endif

/*
 *  Because this file currently only supports platforms which have
 *  precise powers of 2 as bit sizes for the default integers, the
 *  least definitions are all trivial.  Its possible that a future
 *  version of this file could have different definitions.
 */

#ifndef stdint_least_defined
  typedef   int8_t   int_least8_t;
  typedef  uint8_t  uint_least8_t;
  typedef  int16_t  int_least16_t;
  typedef uint16_t uint_least16_t;
  typedef  int32_t  int_least32_t;
  typedef uint32_t uint_least32_t;
# define PRINTF_LEAST32_MODIFIER PRINTF_INT32_MODIFIER
# define PRINTF_LEAST16_MODIFIER PRINTF_INT16_MODIFIER
# define  UINT_LEAST8_MAX  UINT8_MAX
# define   INT_LEAST8_MAX   INT8_MAX
# define UINT_LEAST16_MAX UINT16_MAX
# define  INT_LEAST16_MAX  INT16_MAX
# define UINT_LEAST32_MAX UINT32_MAX
# define  INT_LEAST32_MAX  INT32_MAX
# define   INT_LEAST8_MIN   INT8_MIN
# define  INT_LEAST16_MIN  INT16_MIN
# define  INT_LEAST32_MIN  INT32_MIN
# ifdef stdint_int64_defined
    typedef  int64_t  int_least64_t;
    typedef uint64_t uint_least64_t;
#   define PRINTF_LEAST64_MODIFIER PRINTF_INT64_MODIFIER
#   define UINT_LEAST64_MAX UINT64_MAX
#   define  INT_LEAST64_MAX  INT64_MAX
#   define  INT_LEAST64_MIN  INT64_MIN
# endif
#endif
#undef stdint_least_defined

/*
 *  The ANSI C committee pretending to know or specify anything about
 *  performance is the epitome of misguided arrogance.  The mandate of
 *  this file is to *ONLY* ever support that absolute minimum
 *  definition of the fast integer types, for compatibility purposes.
 *  No extensions, and no attempt to suggest what may or may not be a
 *  faster integer type will ever be made in this file.  Developers are
 *  warned to stay away from these types when using this or any other
 *  stdint.h.
 */

#ifndef int_fast8_t
typedef   int_least8_t   int_fast8_t;
#endif
#ifndef uint_fast8_t
typedef  uint_least8_t  uint_fast8_t;
#endif
#ifndef int_fast16_t
typedef  int_least16_t  int_fast16_t;
#endif
#ifndef uint_fast16_t
typedef uint_least16_t uint_fast16_t;
#endif
#ifndef int_fast32_t
typedef  int_least32_t  int_fast32_t;
#endif
#ifndef uint_fast32_t
typedef uint_least32_t uint_fast32_t;
#endif
#define  UINT_FAST8_MAX  UINT_LEAST8_MAX
#define   INT_FAST8_MAX   INT_LEAST8_MAX
#define UINT_FAST16_MAX UINT_LEAST16_MAX
#define  INT_FAST16_MAX  INT_LEAST16_MAX
#define UINT_FAST32_MAX UINT_LEAST32_MAX
#define  INT_FAST32_MAX  INT_LEAST32_MAX
#define   INT_FAST8_MIN   INT_LEAST8_MIN
#define  INT_FAST16_MIN  INT_LEAST16_MIN
#define  INT_FAST32_MIN  INT_LEAST32_MIN
#ifdef stdint_int64_defined
  typedef  int_least64_t  int_fast64_t;
  typedef uint_least64_t uint_fast64_t;
# define UINT_FAST64_MAX UINT_LEAST64_MAX
# define  INT_FAST64_MAX  INT_LEAST64_MAX
# define  INT_FAST64_MIN  INT_LEAST64_MIN
#endif

#undef stdint_int64_defined

/*
 *  Whatever piecemeal, per compiler thing we can do about the wchar_t
 *  type limits.
 */

#if defined(__WATCOMC__) || defined(_MSC_VER) || defined (__GNUC__)
# include <wchar.h>
# ifndef WCHAR_MIN
#  define WCHAR_MIN 0
# endif
# ifndef WCHAR_MAX
#  define WCHAR_MAX ((wchar_t)-1)
# endif
#endif

/*
 *  Whatever piecemeal, per compiler/platform thing we can do about the
 *  (u)intptr_t types and limits.
 */

#if defined (_MSC_VER) && defined (_UINTPTR_T_DEFINED)
# define STDINT_H_UINTPTR_T_DEFINED
#endif

#ifndef STDINT_H_UINTPTR_T_DEFINED
# if defined (__alpha__) || defined (__ia64__) || defined (__x86_64__) || defined (_WIN64)
#  define stdint_intptr_bits 64
# elif defined (__WATCOMC__) || defined (__TURBOC__)
#  if defined(__TINY__) || defined(__SMALL__) || defined(__MEDIUM__)
#    define stdint_intptr_bits 16
#  else
#    define stdint_intptr_bits 32
#  endif
# elif defined (__i386__) || defined (_WIN32) || defined (WIN32)
#  define stdint_intptr_bits 32
# elif defined (__INTEL_COMPILER)
/* TODO -- what will Intel do about x86-64? */
# endif

# ifdef stdint_intptr_bits
#  define stdint_intptr_glue3_i(a,b,c)  a##b##c
#  define stdint_intptr_glue3(a,b,c)    stdint_intptr_glue3_i(a,b,c)
#  ifndef PRINTF_INTPTR_MODIFIER
#    define PRINTF_INTPTR_MODIFIER      stdint_intptr_glue3(PRINTF_INT,stdint_intptr_bits,_MODIFIER)
#  endif
#  ifndef PTRDIFF_MAX
#    define PTRDIFF_MAX                 stdint_intptr_glue3(INT,stdint_intptr_bits,_MAX)
#  endif
#  ifndef PTRDIFF_MIN
#    define PTRDIFF_MIN                 stdint_intptr_glue3(INT,stdint_intptr_bits,_MIN)
#  endif
#  ifndef UINTPTR_MAX
#    define UINTPTR_MAX                 stdint_intptr_glue3(UINT,stdint_intptr_bits,_MAX)
#  endif
#  ifndef INTPTR_MAX
#    define INTPTR_MAX                  stdint_intptr_glue3(INT,stdint_intptr_bits,_MAX)
#  endif
#  ifndef INTPTR_MIN
#    define INTPTR_MIN                  stdint_intptr_glue3(INT,stdint_intptr_bits,_MIN)
#  endif
#  ifndef INTPTR_C
#    define INTPTR_C(x)                 stdint_intptr_glue3(INT,stdint_intptr_bits,_C)(x)
#  endif
#  ifndef UINTPTR_C
#    define UINTPTR_C(x)                stdint_intptr_glue3(UINT,stdint_intptr_bits,_C)(x)
#  endif
  typedef stdint_intptr_glue3(uint,stdint_intptr_bits,_t) uintptr_t;
  typedef stdint_intptr_glue3( int,stdint_intptr_bits,_t)  intptr_t;
# else
/* TODO -- This following is likely wrong for some platforms, and does
   nothing for the definition of uintptr_t. */
  typedef ptrdiff_t intptr_t;
# endif
# define STDINT_H_UINTPTR_T_DEFINED
#endif

/*
 *  Assumes sig_atomic_t is signed and we have a 2s complement machine.
 */

#ifndef SIG_ATOMIC_MAX
# define SIG_ATOMIC_MAX ((((sig_atomic_t) 1) << (sizeof (sig_atomic_t)*CHAR_BIT-1)) - 1)
#endif

#endif

#ifndef XSENS_MONOLITHIC

#if defined (__TEST_PSTDINT_FOR_CORRECTNESS)

/* 
 *  Please compile with the maximum warning settings to make sure macros are not
 *  defined more than once.
 */
 
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
 
#define glue3_aux(x,y,z) x ## y ## z
#define glue3(x,y,z) glue3_aux(x,y,z)

#define DECLU(bits) glue3(uint,bits,_t) glue3(u,bits,=) glue3(UINT,bits,_C) (0);
#define DECLI(bits) glue3(int,bits,_t) glue3(i,bits,=) glue3(INT,bits,_C) (0);

#define DECL(us,bits) glue3(DECL,us,) (bits)

#define TESTUMAX(bits) glue3(u,bits,=) glue3(~,u,bits); if (glue3(UINT,bits,_MAX) glue3(!=,u,bits)) printf ("Something wrong with UINT%d_MAX\n", bits)
 
int main () {
	DECL(I,8)
	DECL(U,8)
	DECL(I,16)
	DECL(U,16)
	DECL(I,32)
	DECL(U,32)
#ifdef INT64_MAX
	DECL(I,64)
	DECL(U,64)
#endif
	intmax_t imax = INTMAX_C(0);
	uintmax_t umax = UINTMAX_C(0);
	char str0[256], str1[256];

	sprintf (str0, "%d %x\n", 0, ~0);
	
	sprintf (str1, "%d %x\n",  i8, ~0);
	if (0 != strcmp (str0, str1)) printf ("Something wrong with i8 : %s\n", str1);
	sprintf (str1, "%u %x\n",  u8, ~0);
	if (0 != strcmp (str0, str1)) printf ("Something wrong with u8 : %s\n", str1);
	sprintf (str1, "%d %x\n",  i16, ~0);
	if (0 != strcmp (str0, str1)) printf ("Something wrong with i16 : %s\n", str1);
	sprintf (str1, "%u %x\n",  u16, ~0);
	if (0 != strcmp (str0, str1)) printf ("Something wrong with u16 : %s\n", str1);	
	sprintf (str1, "%" PRINTF_INT32_MODIFIER "d %x\n",  i32, ~0);
	if (0 != strcmp (str0, str1)) printf ("Something wrong with i32 : %s\n", str1);
	sprintf (str1, "%" PRINTF_INT32_MODIFIER "u %x\n",  u32, ~0);
	if (0 != strcmp (str0, str1)) printf ("Something wrong with u32 : %s\n", str1);
#ifdef INT64_MAX	
	sprintf (str1, "%" PRINTF_INT64_MODIFIER "d %x\n",  i64, ~0);
	if (0 != strcmp (str0, str1)) printf ("Something wrong with i64 : %s\n", str1);
#endif
	sprintf (str1, "%" PRINTF_INTMAX_MODIFIER "d %x\n",  imax, ~0);
	if (0 != strcmp (str0, str1)) printf ("Something wrong with imax : %s\n", str1);
	sprintf (str1, "%" PRINTF_INTMAX_MODIFIER "u %x\n",  umax, ~0);
	if (0 != strcmp (str0, str1)) printf ("Something wrong with umax : %s\n", str1);	
	
	TESTUMAX(8);
	TESTUMAX(16);
	TESTUMAX(32);
#ifdef INT64_MAX
	TESTUMAX(64);
#endif

	return EXIT_SUCCESS;
}

#endif
#endif // #ifndef XSENS_MONOLITHIC
/*! \file
	\brief	Provides Standard return values for Xsens functions

	\section FileCopyright Copyright Notice
	Copyright (C) Xsens Technologies B.V., 2006.  All rights reserved.

	This source code is intended for use only by Xsens Technologies BV and
	those that have explicit written permission to use it from
	Xsens Technologies BV.

	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
	PARTICULAR PURPOSE.
*/

/*! \cond NODOXYGEN */

#ifndef XSENS_STD_H
#define XSENS_STD_H

//////////////////////////////////////////////////////////////////////////////////////////

#if defined(_WIN32) && !defined(_WIN32_WINNT)
#	define _WIN32_WINNT 0x0502
#endif

#ifdef _WIN32
#	include <windows.h>
#endif

/*! \endcond */

//////////////////////////////////////////////////////////////////////////////////////////
// NOTE!!! When adding a value to this list, also add its description in xsens_std.cpp
/*!	\defgroup enums Global enumerations
	\addtogroup enums
	@{
*/
/*!	\brief Xsens return values
	\details These values are used to signal success or specific failures of functions
*/
enum XsensResultValue {
	// general OK
	 XRV_OK					= 0		//!< 0: Operation was performed successfully

	// communication protocol
	,XRV_NOBUS				= 1		//!< 1: No bus communication possible
	,XRV_BUSNOTREADY		= 2		//!< 2: InitBus and/or SetBID are not issued
	,XRV_INVALIDPERIOD		= 3		//!< 3: Period sent is invalid
	,XRV_INVALIDMSG			= 4		//!< 4: The message is invalid or not implemented
	,XRV_INITBUSFAIL1		= 16	//!< 16: A slave did not respond to WaitForSetBID
	,XRV_INITBUSFAIL2		= 17	//!< 17: An incorrect answer received after WaitForSetBID
	,XRV_INITBUSFAIL3		= 18	//!< 18: After four bus-scans still undetected Motion Trackers
	,XRV_SETBIDFAIL1		= 20	//!< 20: No reply to SetBID message during SetBID procedure
	,XRV_SETBIDFAIL2		= 21	//!< 21: Other than SetBIDAck received
	,XRV_MEASUREMENTFAIL1	= 24	//!< 24: Timer overflow - period too short to collect all data from Motion Trackers
	,XRV_MEASUREMENTFAIL2	= 25	//!< 25: Motion Tracker responds with other than SlaveData message
	,XRV_MEASUREMENTFAIL3	= 26	//!< 26: Total bytes of data of Motion Trackers including sample counter exceeds 255 bytes
	,XRV_MEASUREMENTFAIL4	= 27	//!< 27: Timer overflows during measurement
	,XRV_MEASUREMENTFAIL5	= 28	//!< 28: Timer overflows during measurement
	,XRV_MEASUREMENTFAIL6	= 29	//!< 29: No correct response from Motion Tracker during measurement
	,XRV_TIMEROVERFLOW		= 30	//!< 30: Timer overflows during measurement
	,XRV_BAUDRATEINVALID	= 32	//!< 32: Baud rate does not comply with valid range
	//,XRV_PARAMINVALID		= 33	//!< 33: An invalid parameter is supplied
	,XRV_INVALIDPARAM		= 33	//!< 33: An invalid parameter is supplied
	,XRV_MEASUREMENTFAIL7	= 35	//!< 35: TX PC Buffer is full
	,XRV_MEASUREMENTFAIL8	= 36	//!< 36: TX PC Buffer overflow, cannot fit full message
	,XRV_DEVICEERROR		= 40	//!< 40: The device generated an error, try updating the firmware

	// CMT / XME / etc
	,XRV_ERROR					= 256	//!< 256: A generic error occurred
	,XRV_NOTIMPLEMENTED			= 257	//!< 257: Operation not implemented in this version (yet)
	,XRV_TIMEOUT				= 258	//!< 258: A timeout occurred
	,XRV_TIMEOUTNODATA			= 259	//!< 259: Operation aborted because of no data read
	,XRV_CHECKSUMFAULT			= 260	//!< 260: Checksum fault occurred
	,XRV_OUTOFMEMORY			= 261	//!< 261: No internal memory available
	,XRV_NOTFOUND				= 262	//!< 262: The requested item was not found
	,XRV_UNEXPECTEDMSG			= 263	//!< 263: Unexpected message received (e.g. no acknowledge message received)
	,XRV_INVALIDID				= 264	//!< 264: Invalid id supplied
	,XRV_INVALIDOPERATION		= 265		//!< 265: Operation is invalid at this point
	,XRV_INSUFFICIENTSPACE		= 266	//!< 266: Insufficient buffer space available
	,XRV_INPUTCANNOTBEOPENED	= 267	//!< 267: The specified i/o device can not be opened
	,XRV_OUTPUTCANNOTBEOPENED	= 268	//!< 268: The specified i/o device can not be opened
	,XRV_ALREADYOPEN			= 269	//!< 269: An I/O device is already opened with this object
	,XRV_ENDOFFILE				= 270	//!< 270: End of file is reached
	,XRV_COULDNOTREADSETTINGS	= 271	//!< 271: A required settings file could not be opened or is missing some data
	,XRV_NODATA					= 272	//!< 272: No data is available
	,XRV_READONLY				= 273	//!< 273: Tried to change a read-only value
	,XRV_NULLPTR				= 274	//!< 274: Tried to supply a NULL value where it is not allowed
	,XRV_INSUFFICIENTDATA		= 275	//!< 275: Insufficient data was supplied to a function
	,XRV_BUSY					= 276	//!< 276: Busy processing, try again later
	,XRV_INVALIDINSTANCE		= 277	//!< 277: Invalid instance called
	,XRV_DATACORRUPT			= 278	//!< 278: A trusted data stream proves to contain corrupted data

	,XRV_READINITFAILED			= 279	//!< 279: Failure during read of settings
	,XRV_NOXMFOUND				= 280	//!< 280: Could not find any MVN-compatible hardware
	,XRV_ONLYONEXMFOUND			= 281	//!< 281: Found only one responding Xbus Master
	,XRV_MTCOUNTZERO			= 282	//!< 282: No sensors found
	,XRV_MTLOCATIONINVALID		= 283	//!< 283: One or more sensors are not where they were expected
	,XRV_INSUFFICIENTMTS		= 284	//!< 284: Not enough sensors were found
	,XRV_INITFUSIONFAILED		= 285	//!< 285: Failure during initialization of Fusion Engine
	,XRV_OTHER					= 286	//!< 286: Something else was received than was requested

	,XRV_NOFILEOPEN				= 287	//!< 287: No file opened for reading/writing
	,XRV_NOPORTOPEN				= 288	//!< 288: No serial port opened for reading/writing
	,XRV_NOFILEORPORTOPEN		= 289	//!< 289: No file or serial port opened for reading/writing
	,XRV_PORTNOTFOUND			= 290	//!< 290: A required port could not be found
	,XRV_INITPORTFAILED			= 291	//!< 291: The low-level port handler failed to initialize
	,XRV_CALIBRATIONFAILED		= 292	//!< 292: A calibration routine failed

	,XRV_CONFIGCHECKFAIL		= 293	//!< 293: The in-config check of the device failed
	,XRV_ALREADYDONE			= 294	//!< 294: The operation is once only and has already been performed

	,XRV_SYNC_SINGLE_SLAVE		= 295	//!< 295: The single connected device is configured as a slave
	,XRV_SYNC_SECOND_MASTER		= 296	//!< 296: More than one master was detected
	,XRV_SYNC_NO_SYNC			= 297	//!< 297: A device was detected that was neither master nor slave
	,XRV_SYNC_NO_MASTER			= 298	//!< 298: No master detected
	,XRV_SYNC_DATA_MISSING		= 299	//!< 299: A device is not sending enough data

	,XRV_VERSION_TOO_LOW		= 300	//!< 300: The version of the object is too low for the requested operation
	,XRV_VERSION_PROBLEM		= 301	//!< 301: The object has an unrecognized version, so it's not safe to perform the operation

	,XRV_ABORTED				= 302	//!< 302: The process was aborted by an external event, usually a user action or process termination
};
//! @}

//////////////////////////////////////////////////////////////////////////////////////////

/*! \cond NODOXYGEN */
#ifndef DELNUL
//! This macro deletes a pointer and sets it to NULL
#define DELNUL(ptr)		do { delete ptr; ptr = NULL; } while (0);
//! This macro deletes a pointer if it is not NULL and then sets it to NULL
#define CHKDELNUL(ptr)	do { if (ptr != NULL) { delete ptr; ptr = NULL; } } while (0);
//! This macro deletes a multi-object pointer and sets it to NULL
#define LSTDELNUL(ptr)		do { delete[] ptr; ptr = NULL; } while (0);
//! This macro deletes a multi-object pointer if it is not NULL and then sets it to NULL
#define LSTCHKDELNUL(ptr)	do { if (ptr != NULL) { delete[] ptr; ptr = NULL; } } while (0);

//! This macro frees a pointer and then sets it to NULL
#define FREENUL(ptr)	do { free(ptr); ptr = NULL; } while (0);
//! This macro frees a pointer if it is not NULL and then sets it to NULL
#define CHKFREENUL(ptr)	do { if (ptr != NULL) { free(ptr); ptr = NULL; } } while (0);
#endif
/*! \endcond */

//////////////////////////////////////////////////////////////////////////////////////////

/* \brief Retrieve a string corresponding to the given result code.

	This function uses the CmtResultText list to return a string with the relevant result 
	code in a textual format. If the supplied result code is invalid the
	"!!Invalid result code!!" string is returned.
*/
#if !defined(_CMT_DLL_IMPORT) && !defined(_XME_DLL_IMPORT)
const char* xsensResultText(const XsensResultValue result);
#endif

//////////////////////////////////////////////////////////////////////////////////////////

#define XSENS_LABEL_SIZE			256		//!< The maximum length of a label
#define XSENS_SHORT_STRING_SIZE		256		//!< The maximum length of a short string
#define XSENS_LONG_STRING_SIZE		16384	//!< The maximum length of a long string

//////////////////////////////////////////////////////////////////////////////////////////

/*! \cond NODOXYGEN */
#endif	// XSENS_STD_H
/*! \endcond */

#ifndef XSENS_FILE_H
#define XSENS_FILE_H

#include <stdio.h>
#ifndef XSENS_MONOLITHIC
#ifndef _PSTDINT_H_INCLUDED
#	include "pstdint.h"
#endif
#endif

#ifndef _WIN32
#   include <sys/types.h>
#endif

#ifdef _WIN32
	typedef  __int64	XsensFilePos;
#else
	/* off_t is practically guaranteed not to be 64 bits on non64 bit systems.
	   We'd better explicitly use __off64_t to be sure of it's size.
	*/
	typedef  __off64_t		XsensFilePos;
#endif

// setFilePos defines
#ifdef _WIN32
#	define XSENS_FILEPOS_BEGIN				FILE_BEGIN
#	define XSENS_FILEPOS_CURRENT			FILE_CURRENT
#	define XSENS_FILEPOS_END				FILE_END
#else
#	define XSENS_FILEPOS_BEGIN				SEEK_SET
#	define XSENS_FILEPOS_CURRENT			SEEK_CUR
#	define XSENS_FILEPOS_END				SEEK_END
#endif

#endif	// XSENS_FILE_H
/*! \file
	\brief	Macros and types for use in the Xsens communication protocol and CMT classes

	This file contains useful macros for the Xsens CMT classes. These include
	Message Ids, sizes of messages, result/error codes, etc.
	
	The macros and enumerators in this file are either used on multiple levels (XsensResultValue)
	or so numerous (CMT_MID_...) that it would not be justifiable to put them in a header
	file of a particular level.

	\section FileCopyright Copyright Notice 
	Copyright (C) Xsens Technologies B.V., 2006.  All rights reserved.

	This source code is intended for use only by Xsens Technologies BV and
	those that have explicit written permission to use it from
	Xsens Technologies BV.

	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
	PARTICULAR PURPOSE.

	\section FileChangelog	Changelog
	\par 2006-05-01, v0.0.1
	\li Job Mulder:	Created Cmtdef.h
	\par 2006-07-21, v0.1.0
	\li Job Mulder:	Updated file for release 0.1.0
*/

#ifndef CMTDEF_H
#define CMTDEF_H

#ifndef XSENS_MONOLITHIC
#ifndef _PSTDINT_H_INCLUDED
#	include "pstdint.h"
#endif
#endif
#include <time.h>

#ifdef _WIN32
#else
#	include <termios.h>
#	include <string.h>
#	ifndef	B460800
#		undef	B230400
#		define	B230400	0010003
#		define  B460800 0010004
#		define  B921600 0010007
#	endif
// required for gnu c++ compiler due to difference in attribute declarations
#   define __cdecl __attribute__((cdecl))
#   define __stdcall __attribute__((stdcall))
#endif

#ifndef XSENS_STD_H
#	include "xsens_std.h"
#endif
#if !defined(XSENS_TIME_H) &&!defined(CMT_DLL_IMPORT)
#	include "xsens_time.h"
#endif

#ifndef XSENS_FILE_H
#	include "xsens_file.h"
#endif

#ifdef __cplusplus
#include <string.h>
#endif

//////////////////////////////////////////////////////////////////////////////////////////
// Field message indices
#define CMT_IND_PREAMBLE				0
#define CMT_IND_BID						1
#define CMT_IND_MID						2
#define CMT_IND_LEN						3
#define CMT_IND_DATA0					4
#define CMT_IND_LENEXTH					4
#define CMT_IND_LENEXTL					5
#define CMT_IND_DATAEXT0				6

#define CMT_PREAMBLE					0xFA
#define CMT_BID_MASTER					0xFF
#define CMT_BID_MT						0x01
#define CMT_EXTLENCODE					0xFF

#define CMT_LEN_MSGHEADER				4
#define CMT_LEN_MSGEXTHEADER			6
#define CMT_LEN_MSGHEADERCS				5
#define CMT_LEN_MSGEXTHEADERCS			7
#define CMT_LEN_CHECKSUM				1
#define CMT_LEN_UNSIGSHORT				2
#define CMT_LEN_UNSIGINT				4
#define CMT_LEN_FLOAT					4

// Maximum message/data length
#define CMT_MAXDATALEN					8192
#define CMT_MAXSHORTDATALEN				254
#define CMT_MAXMSGLEN					(CMT_MAXDATALEN+7)
#define CMT_MAXSHORTMSGLEN				(CMT_MAXSHORTDATALEN+5)
#define CMT_MAXGARBAGE					(CMT_MAXMSGLEN+1)


// DID Type (high nibble)
#define CMT_DID_TYPEH_MASK				0x00F00000
#define CMT_DID_TYPEL_MASK				0x000F0000
#define CMT_DID_TYPEH_MT				0x00000000
#define CMT_DID_TYPEH_XM				0x00100000
#define CMT_DID_TYPEH_MTI_MTX			0x00300000
#define CMT_DID_TYPEH_MTIG				0x00500000
#define CMT_DID_TYPE_MASK				0x00FF0000
#define CMT_DID_TYPE_MASK_MT_232		0x00300000
#define CMT_DID_TYPE_MASK_MT_422		0x00310000
#define CMT_DID_TYPE_MASK_MT_XM			0x00020000
#define CMT_DID_TYPE_MASK_MT_485		0x00330000

// All Message identifiers
// WakeUp state messages
#define CMT_MID_WAKEUP					0x3E
#define CMT_MID_WAKEUPACK				0x3F

// Config state messages
#define CMT_MID_REQDID					0x00
#define CMT_MID_DEVICEID				0x01
#define CMT_LEN_DEVICEID				4
#define CMT_MID_INITBUS					0x02
#define CMT_MID_INITBUSRESULTS			0x03
#define CMT_LEN_INITBUSRESULTS			4
#define CMT_MID_REQPERIOD				0x04
#define CMT_MID_REQPERIODACK			0x05
#define CMT_LEN_PERIOD					2
#define CMT_MID_SETPERIOD				0x04
#define CMT_MID_SETPERIODACK			0x05
// XbusMaster
#define CMT_MID_SETBID					0x06
#define CMT_MID_SETBIDACK				0x07
#define CMT_MID_AUTOSTART				0x06
#define CMT_MID_AUTOSTARTACK			0x07
#define CMT_MID_BUSPWR					0x08
#define CMT_LEN_BUSPWR					2
#define CMT_MID_BUSPWRACK				0x09
// End XbusMaster
#define CMT_MID_REQDATALENGTH			0x0A
#define CMT_MID_DATALENGTH				0x0B
#define CMT_LEN_DATALENGTH				2
#define CMT_MID_REQCONFIGURATION		0x0C
#define CMT_MID_CONFIGURATION			0x0D
#define CMT_LEN_CONFIGURATION			118
#define CMT_MID_RESTOREFACTORYDEF		0x0E
#define CMT_MID_RESTOREFACTORYDEFACK	0x0F

#define CMT_MID_GOTOMEASUREMENT			0x10
#define CMT_MID_GOTOMEASUREMENTACK		0x11
#define CMT_MID_REQFWREV				0x12
#define CMT_MID_FIRMWAREREV				0x13
#define CMT_LEN_FIRMWAREREV				3
// XbusMaster
#define CMT_MID_REQBTDISABLE			0x14
#define CMT_MID_REQBTDISABLEACK			0x15
#define CMT_LEN_BTDISABLE				1
#define CMT_MID_DISABLEBT				0x14
#define CMT_MID_DISABLEBTACK			0x15
#define CMT_MID_REQOPMODE				0x16
#define CMT_MID_REQOPMODEACK			0x17
#define CMT_LEN_OPMODE					1
#define CMT_MID_SETOPMODE				0x16
#define CMT_MID_SETOPMODEACK			0x17
// End XbusMaster
#define CMT_MID_REQBAUDRATE				0x18
#define CMT_MID_REQBAUDRATEACK			0x19
#define CMT_LEN_BAUDRATE				1
#define CMT_MID_SETBAUDRATE				0x18
#define CMT_MID_SETBAUDRATEACK			0x19
// XbusMaster
#define CMT_MID_REQSYNCMODE				0x1A
#define CMT_MID_REQSYNCMODEACK			0x1B
#define CMT_LEN_SYNCMODE				1
#define CMT_MID_SETSYNCMODE				0x1A
#define CMT_MID_SETSYNCMODEACK			0x1B
// End XbusMaster
#define CMT_MID_REQPRODUCTCODE			0x1C
#define CMT_MID_PRODUCTCODE				0x1D
#define CMT_LEN_PRODUCTCODE				20

#define CMT_MID_REQPROCESSINGFLAGS		0x20
#define CMT_MID_REQPROCESSINGFLAGSACK	0x21
#define CMT_LEN_PROCESSINGFLAGS			2
#define CMT_MID_SETPROCESSINGFLAGS		0x20
#define CMT_MID_SETPROCESSINGFLAGSACK	0x21

// XbusMaster
#define CMT_MID_XMPWROFF				0x44
#define CMT_LEN_XMPWROFF				0
// End XbusMaster

#define CMT_MID_REQOUTPUTMODE			0xD0
#define CMT_MID_REQOUTPUTMODEACK		0xD1
#define CMT_LEN_OUTPUTMODE		 		2
#define CMT_MID_SETOUTPUTMODE			0xD0
#define CMT_MID_SETOUTPUTMODEACK		0xD1

#define CMT_MID_REQOUTPUTSETTINGS		0xD2
#define CMT_MID_REQOUTPUTSETTINGSACK	0xD3
#define CMT_LEN_OUTPUTSETTINGS		 	4
#define CMT_MID_SETOUTPUTSETTINGS		0xD2
#define CMT_MID_SETOUTPUTSETTINGSACK	0xD3

#define CMT_MID_REQOUTPUTSKIPFACTOR		0xD4
#define CMT_MID_REQOUTPUTSKIPFACTORACK	0xD5
#define CMT_LEN_OUTPUTSKIPFACTOR		2
#define CMT_MID_SETOUTPUTSKIPFACTOR		0xD4
#define CMT_MID_SETOUTPUTSKIPFACTORACK	0xD5

#define CMT_MID_REQSYNCINSETTINGS		0xD6
#define CMT_MID_REQSYNCINSETTINGSACK	0xD7
#define CMT_LEN_SYNCINMODE				2
#define CMT_LEN_SYNCINSKIPFACTOR		2
#define CMT_LEN_SYNCINOFFSET			4
#define CMT_MID_SETSYNCINSETTINGS		0xD6
#define CMT_MID_SETSYNCINSETTINGSACK	0xD7

#define CMT_MID_REQSYNCOUTSETTINGS		0xD8
#define CMT_MID_REQSYNCOUTSETTINGSACK	0xD9
#define CMT_LEN_SYNCOUTMODE				2
#define CMT_LEN_SYNCOUTSKIPFACTOR		2
#define CMT_LEN_SYNCOUTOFFSET			4
#define CMT_LEN_SYNCOUTPULSEWIDTH		4
#define CMT_MID_SETSYNCOUTSETTINGS		0xD8
#define CMT_MID_SETSYNCOUTSETTINGSACK	0xD9

#define CMT_MID_REQERRORMODE			0xDA
#define CMT_MID_REQERRORMODEACK			0xDB
#define CMT_LEN_ERRORMODE				2
#define CMT_MID_SETERRORMODE			0xDA
#define CMT_MID_SETERRORMODEACK			0xDB

#define CMT_MID_REQTRANSMITDELAY		0xDC
#define CMT_MID_REQTRANSMITDELAYACK		0xDD
#define CMT_LEN_TRANSMITDELAY			2
#define CMT_MID_SETTRANSMITDELAY		0xDC
#define CMT_MID_SETTRANSMITDELAYACK		0xDD	

#define CMT_MID_REQOBJECTALIGNMENT		0xE0	
#define CMT_MID_REQOBJECTALIGNMENTACK	0xE1
#define CMT_LEN_OBJECTALIGNMENT			36
#define CMT_MID_SETOBJECTALIGNMENT		0xE0	
#define CMT_MID_SETOBJECTALIGNMENTACK	0xE1

// Xbus Master
#define CMT_MID_REQXMERRORMODE			0x82
#define CMT_MID_REQXMERRORMODEACK		0x83
#define CMT_LEN_XMERRORMODE				2
#define CMT_MID_SETXMERRORMODE			0x82
#define CMT_MID_SETXMERRORMODEACK		0x83

#define CMT_MID_REQBUFFERSIZE			0x84
#define CMT_MID_REQBUFFERSIZEACK		0x85
#define CMT_LEN_BUFFERSIZE				2
#define CMT_MID_SETBUFFERSIZE			0x84
#define CMT_MID_SETBUFFERSIZEACK		0x85			
// End Xbus Master

#define CMT_MID_REQHEADING				0x82
#define CMT_MID_REQHEADINGACK			0x83
#define CMT_LEN_HEADING		 			4
#define CMT_MID_SETHEADING				0x82
#define CMT_MID_SETHEADINGACK			0x83

#define CMT_MID_REQMAGNETICDECLINATION		0x6A
#define CMT_MID_REQMAGNETICDECLINATIONACK	0x6B
#define CMT_LEN_MAGNETICDECLINATION		 	4
#define CMT_MID_SETMAGNETICDECLINATION		0x6A
#define CMT_MID_SETMAGNETICDECLINATIONACK	0x6B

#define CMT_MID_REQLOCATIONID			0x84
#define CMT_MID_REQLOCATIONIDACK		0x85
#define CMT_LEN_LOCATIONID				2
#define CMT_MID_SETLOCATIONID			0x84
#define CMT_MID_SETLOCATIONIDACK		0x85

#define CMT_MID_REQEXTOUTPUTMODE		0x86
#define CMT_MID_REQEXTOUTPUTMODEACK		0x87
#define CMT_LEN_EXTOUTPUTMODE			2
#define CMT_MID_SETEXTOUTPUTMODE		0x86
#define CMT_MID_SETEXTOUTPUTMODEACK		0x87

// XbusMaster
#define CMT_MID_REQBATLEVEL				0x88
#define CMT_MID_BATLEVEL				0x89
// End XbusMaster

#define CMT_MID_REQINITTRACKMODE		0x88
#define CMT_MID_REQINITTRACKMODEACK		0x89
#define CMT_LEN_INITTRACKMODE			2
#define CMT_MID_SETINITTRACKMODE		0x88
#define CMT_MID_SETINITTRACKMODEACK		0x89

#define CMT_MID_STOREXKFSTATE			0x8A
#define CMT_LEN_STOREXKFSTATE			0
#define CMT_MID_STOREXKFSTATEACK		0x8B

#define CMT_MID_REQUTCTIME				0x60
#define CMT_MID_UTCTIME					0x61
#define CMT_LEN_UTCTIME					12

#define CMT_LEN_SCENARIOLABEL			20
#define CMT_LEN_SCENARIOFULL			(1+1+CMT_LEN_SCENARIOLABEL)
#define	CMT_MID_REQAVAILABLESCENARIOS	0x62
#define CMT_MID_AVAILABLESCENARIOS		0x63
#define CMT_LEN_AVAILABLESCENARIOS		(CMT_MAX_SCENARIOS_IN_MT*CMT_LEN_SCENARIOFULL)

#define CMT_MID_REQSCENARIO				0x64
#define CMT_MID_REQSCENARIOACK			0x65
#define CMT_LEN_REQSCENARIOACK			2
#define CMT_MID_SETSCENARIO				0x64
#define CMT_MID_SETSCENARIOACK			0x65
#define CMT_LEN_SETSCENARIO				1

#define CMT_MID_REQGRAVITYMAGNITUDE		0x66
#define CMT_MID_REQGRAVITYMAGNITUDEACK	0x67
#define CMT_MID_SETGRAVITYMAGNITUDE		0x66
#define CMT_MID_SETGRAVITYMAGNITUDEACK	0x67
#define CMT_LEN_GRAVITYMAGNITUDE		4

#define CMT_MID_REQGPSLEVERARM			0x68
#define CMT_MID_REQGPSLEVERARMACK		0x69
#define CMT_MID_SETGPSLEVERARM			0x68
#define CMT_MID_SETGPSLEVERARMACK		0x69
#define CMT_LEN_GPSLEVERARM				12

#define CMT_MID_REQLATLONALT			0x6E
#define CMT_MID_REQLATLONALTACK			0x6F
#define CMT_LEN_LATLONALT				18
#define CMT_MID_SETLATLONALT			0x6E
#define CMT_MID_SETLATLONALTACK			0x6F

// Measurement state
#define CMT_MID_GOTOCONFIG				0x30
#define CMT_MID_GOTOCONFIGACK			0x31
#define CMT_MID_BUSDATA					0x32
#define CMT_MID_MTDATA					0x32

#define CMT_MID_SETNOROTATION			0x22
#define CMT_MID_SETNOROTATIONACK		0x23
#define CMT_LEN_SETNOROTATION			2

#define CMT_MID_RUNSELFTEST				0x24
#define CMT_MID_SELFTESTRESULTS			0x25
#define CMT_SELFTEST_OK					0x1FF

// Manual
#define CMT_MID_PREPAREDATA				0x32
#define CMT_MID_REQDATA					0x34
#define CMT_MID_REQDATAACK				0x35

// MTData defines 
// Length of data blocks in bytes
#define CMT_LEN_RAWDATA					20
#define CMT_LEN_CALIBDATA				36
#define CMT_LEN_CALIB_ACCDATA			12
#define CMT_LEN_CALIB_GYRDATA			12
#define CMT_LEN_CALIB_MAGDATA			12
#define CMT_LEN_ORIENT_QUATDATA			16
#define CMT_LEN_ORIENT_EULERDATA		12
#define CMT_LEN_ORIENT_MATRIXDATA		36
#define CMT_LEN_SAMPLECNT				2
#define CMT_LEN_TEMPDATA				4

// Length of data blocks in floats
#define CMT_LEN_CALIBDATA_FLT			9
#define CMT_LEN_TEMPDATA_FLT			1
#define CMT_LEN_ORIENT_QUATDATA_FLT		4
#define CMT_LEN_ORIENT_EULERDATA_FLT	3
#define CMT_LEN_ORIENT_MATRIXDATA_FLT	9

// Indices of fields in DATA field of MTData message in bytes
// use in combination with LEN_CALIB etc
// Un-calibrated raw data
#define CMT_IND_RAW_ACCX				0
#define CMT_IND_RAW_ACCY				2
#define CMT_IND_RAW_ACCZ				4
#define CMT_IND_RAW_GYRX				6
#define CMT_IND_RAW_GYRY				8
#define CMT_IND_RAW_GYRZ				10
#define CMT_IND_RAW_MAGX				12
#define CMT_IND_RAW_MAGY				14
#define CMT_IND_RAW_MAGZ				16
#define CMT_IND_RAW_TEMP				18
// Calibrated data
#define CMT_IND_CALIB_ACCX				0
#define CMT_IND_CALIB_ACCY				4
#define CMT_IND_CALIB_ACCZ				8
#define CMT_IND_CALIB_GYRX				12
#define CMT_IND_CALIB_GYRY				16
#define CMT_IND_CALIB_GYRZ				20
#define CMT_IND_CALIB_MAGX				24
#define CMT_IND_CALIB_MAGY				28
#define CMT_IND_CALIB_MAGZ				32
// Orientation data - quat
#define CMT_IND_ORIENT_Q0				0
#define CMT_IND_ORIENT_Q1				4
#define CMT_IND_ORIENT_Q2				8
#define CMT_IND_ORIENT_Q3				12
// Orientation data - euler
#define CMT_IND_ORIENT_ROLL				0
#define CMT_IND_ORIENT_PITCH			4
#define CMT_IND_ORIENT_YAW				8
// Orientation data - matrix
#define CMT_IND_ORIENT_A				0
#define CMT_IND_ORIENT_B				4
#define CMT_IND_ORIENT_C				8
#define CMT_IND_ORIENT_D				12
#define CMT_IND_ORIENT_E				16
#define CMT_IND_ORIENT_F				20
#define CMT_IND_ORIENT_G				24
#define CMT_IND_ORIENT_H				28
#define CMT_IND_ORIENT_I				32
// Orientation data - euler
#define CMT_IND_SAMPLECNTH				0
#define CMT_IND_SAMPLECNTL				1

// Indices of fields in DATA field of MTData message
// Un-calibrated raw data
#define CMT_FLDNUM_RAW_ACCX				0
#define CMT_FLDNUM_RAW_ACCY				1
#define CMT_FLDNUM_RAW_ACCZ				2
#define CMT_FLDNUM_RAW_GYRX				3
#define CMT_FLDNUM_RAW_GYRY				4
#define CMT_FLDNUM_RAW_GYRZ				5
#define CMT_FLDNUM_RAW_MAGX				6
#define CMT_FLDNUM_RAW_MAGY				7
#define CMT_FLDNUM_RAW_MAGZ				8
#define CMT_FLDNUM_RAW_TEMP				9
// Calibrated data
#define CMT_FLDNUM_CALIB_ACCX			0
#define CMT_FLDNUM_CALIB_ACCY			1
#define CMT_FLDNUM_CALIB_ACCZ			2
#define CMT_FLDNUM_CALIB_GYRX			3
#define CMT_FLDNUM_CALIB_GYRY			4
#define CMT_FLDNUM_CALIB_GYRZ			5
#define CMT_FLDNUM_CALIB_MAGX			6
#define CMT_FLDNUM_CALIB_MAGY			7
#define CMT_FLDNUM_CALIB_MAGZ			8
// Orientation data - quat
#define CMT_FLDNUM_ORIENT_Q0			0
#define CMT_FLDNUM_ORIENT_Q1			1
#define CMT_FLDNUM_ORIENT_Q2			2
#define CMT_FLDNUM_ORIENT_Q3			3
// Orientation data - euler
#define CMT_FLDNUM_ORIENT_ROLL			0
#define CMT_FLDNUM_ORIENT_PITCH			1
#define CMT_FLDNUM_ORIENT_YAW			2
// Orientation data - matrix
#define CMT_FLDNUM_ORIENT_A				0
#define CMT_FLDNUM_ORIENT_B				1
#define CMT_FLDNUM_ORIENT_C				2
#define CMT_FLDNUM_ORIENT_D				3
#define CMT_FLDNUM_ORIENT_E				4
#define CMT_FLDNUM_ORIENT_F				5
#define CMT_FLDNUM_ORIENT_G				6
#define CMT_FLDNUM_ORIENT_H				7
#define CMT_FLDNUM_ORIENT_I				8
// Length
// Uncalibrated raw data
#define CMT_LEN_RAW_ACCX				2
#define CMT_LEN_RAW_ACCY				2
#define CMT_LEN_RAW_ACCZ				2
#define CMT_LEN_RAW_GYRX				2
#define CMT_LEN_RAW_GYRY				2
#define CMT_LEN_RAW_GYRZ				2
#define CMT_LEN_RAW_MAGX				2
#define CMT_LEN_RAW_MAGY				2
#define CMT_LEN_RAW_MAGZ				2
#define CMT_LEN_RAW_TEMP				2
// Calibrated data
#define CMT_LEN_CALIB_ACCX				4
#define CMT_LEN_CALIB_ACCY				4
#define CMT_LEN_CALIB_ACCZ				4
#define CMT_LEN_CALIB_GYRX				4
#define CMT_LEN_CALIB_GYRY				4
#define CMT_LEN_CALIB_GYRZ				4
#define CMT_LEN_CALIB_MAGX				4
#define CMT_LEN_CALIB_MAGY				4
#define CMT_LEN_CALIB_MAGZ				4
// Orientation data - quat
#define CMT_LEN_ORIENT_Q0				4
#define CMT_LEN_ORIENT_Q1				4
#define CMT_LEN_ORIENT_Q2				4
#define CMT_LEN_ORIENT_Q3				4
// Orientation data - euler
#define CMT_LEN_ORIENT_ROLL				4
#define CMT_LEN_ORIENT_PITCH			4
#define CMT_LEN_ORIENT_YAW				4
// Orientation data - matrix
#define CMT_LEN_ORIENT_A				4
#define CMT_LEN_ORIENT_B				4
#define CMT_LEN_ORIENT_C				4
#define CMT_LEN_ORIENT_D				4
#define CMT_LEN_ORIENT_E				4
#define CMT_LEN_ORIENT_F				4
#define CMT_LEN_ORIENT_G				4
#define CMT_LEN_ORIENT_H				4
#define CMT_LEN_ORIENT_I				4

// Defines for getDataValue
#define CMT_VALUE_RAW_ACC				0
#define CMT_VALUE_RAW_GYR				1
#define CMT_VALUE_RAW_MAG				2
#define CMT_VALUE_RAW_TEMP				3
#define CMT_VALUE_CALIB_ACC				4
#define CMT_VALUE_CALIB_GYR				5
#define CMT_VALUE_CALIB_MAG				6
#define CMT_VALUE_ORIENT_QUAT			7
#define CMT_VALUE_ORIENT_EULER			8
#define CMT_VALUE_ORIENT_MATRIX			9
#define CMT_VALUE_SAMPLECNT				10
#define CMT_VALUE_TEMP					11

#define CMT_INVALIDSETTINGVALUE			0xFFFFFFFF

// Valid in all states
#define CMT_MID_RESET					0x40
#define CMT_MID_RESETACK				0x41
#define CMT_MID_ERROR					0x42
// XbusMaster
#define CMT_MID_XMPWROFF				0x44
// End XbusMaster

#define CMT_MID_REQFILTERSETTINGS		0xA0
#define CMT_MID_REQFILTERSETTINGSACK	0xA1
#define CMT_LEN_FILTERSETTINGS			4
#define CMT_MID_SETFILTERSETTINGS		0xA0
#define CMT_MID_SETFILTERSETTINGSACK	0xA1
#define CMT_MID_REQAMD					0xA2
#define CMT_MID_REQAMDACK				0xA3
#define CMT_LEN_AMD						2
#define CMT_MID_SETAMD					0xA2
#define CMT_MID_SETAMDACK				0xA3
#define CMT_MID_RESETORIENTATION		0xA4
#define CMT_MID_RESETORIENTATIONACK		0xA5
#define CMT_LEN_RESETORIENTATION		2

#define CMT_MID_REQGPSSTATUS			0xA6
#define CMT_MID_GPSSTATUS				0xA7
#define CMT_LEN_GPSSTATUS				(1+5*16)

// Baudrate defines for SetBaudrate message
#define CMT_BAUDCODE_4K8				0x0B
#define CMT_BAUDCODE_9K6				0x09
//	#define CMT_BAUDCODE_14K4				0x08
#define CMT_BAUDCODE_19K2				0x07
//	#define CMT_BAUDCODE_28K8				0x06
#define CMT_BAUDCODE_38K4				0x05
#define CMT_BAUDCODE_57K6				0x04
//	#define CMT_BAUDCODE_76K8				0x03
#define CMT_BAUDCODE_115K2				0x02
#define CMT_BAUDCODE_230K4				0x01
#define CMT_BAUDCODE_460K8				0x00
#define CMT_BAUDCODE_921K6_ALT			0x0A		// only usable from FW 2.4.6
#define CMT_BAUDCODE_921K6				0x80

// Xbus protocol error codes (Error)
#define CMT_ERROR_NOBUSCOMM				0x01
#define CMT_ERROR_BUSNOTREADY			0x02
#define CMT_ERROR_PERIODINVALID			0x03
#define CMT_ERROR_MESSAGEINVALID		0x04
#define CMT_ERROR_INITOFBUSFAILED1		0x10
#define CMT_ERROR_INITOFBUSFAILED2		0x11
#define CMT_ERROR_INITOFBUSFAILED3		0x12
#define CMT_ERROR_SETBIDPROCFAILED1		0x14
#define CMT_ERROR_SETBIDPROCFAILED2		0x15
#define CMT_ERROR_MEASUREMENTFAILED1	0x18
#define CMT_ERROR_MEASUREMENTFAILED2	0x19
#define CMT_ERROR_MEASUREMENTFAILED3	0x1A
#define CMT_ERROR_MEASUREMENTFAILED4	0x1B
#define CMT_ERROR_MEASUREMENTFAILED5	0x1C
#define CMT_ERROR_MEASUREMENTFAILED6	0x1D
#define CMT_ERROR_TIMEROVERFLOW			0x1E
#define CMT_ERROR_BAUDRATEINVALID		0x20
#define CMT_ERROR_PARAMETERINVALID		0x21
#define CMT_ERROR_MEASUREMENTFAILED7	0x23

// Error modes (SetErrorMode)
#define CMT_ERRORMODE_IGNORE					0x0000
#define CMT_ERRORMODE_INCSAMPLECNT				0x0001
#define CMT_ERRORMODE_INCSAMPLECNT_SENDERROR	0x0002
#define CMT_ERRORMODE_SENDERROR_GOTOCONFIG		0x0003

// Configuration message defines
#define CMT_CONF_MASTERDID				0
#define CMT_CONF_PERIOD					4
#define CMT_CONF_OUTPUTSKIPFACTOR		6
#define CMT_CONF_SYNCIN_MODE			8
#define CMT_CONF_SYNCIN_SKIPFACTOR		10
#define CMT_CONF_SYNCIN_OFFSET			12
#define CMT_CONF_DATE					16
#define CMT_CONF_TIME					24
#define CMT_CONF_NUMDEVICES				96
// Configuration sensor block properties
#define CMT_CONF_DID					98
#define CMT_CONF_DATALENGTH				102
#define CMT_CONF_OUTPUTMODE				104
#define CMT_CONF_OUTPUTSETTINGS			106
#define CMT_CONF_BLOCKLEN				20
// To calculate the offset in data field for output mode of sensor #2 use
//		CONF_OUTPUTMODE + 1*CONF_BLOCKLEN
#define CMT_CONF_MASTERDIDLEN			4
#define CMT_CONF_PERIODLEN				2
#define CMT_CONF_OUTPUTSKIPFACTORLEN	2
#define CMT_CONF_SYNCIN_MODELEN			2
#define CMT_CONF_SYNCIN_SKIPFACTORLEN	2
#define CMT_CONF_SYNCIN_OFFSETLEN		4
#define CMT_CONF_DATELEN				8
#define CMT_CONF_TIMELEN				8
#define CMT_CONF_RESERVED_CLIENTLEN		32
#define CMT_CONF_RESERVED_HOSTLEN		32
#define CMT_CONF_NUMDEVICESLEN			2
// Configuration sensor block properties
#define CMT_CONF_DIDLEN					4
#define CMT_CONF_DATALENGTHLEN			2
#define CMT_CONF_OUTPUTMODELEN			2
#define CMT_CONF_OUTPUTSETTINGSLEN		4

// Clock frequency for offset & pulse width
#define CMT_SYNC_CLOCKFREQMHZ			29.4912
#define CMT_SYNC_CLOCK_NS_TO_TICKS		(CMT_SYNC_CLOCKFREQMHZ * 1.0e-3)
#define CMT_SYNC_CLOCK_TICKS_TO_NS		(1.0e3 / CMT_SYNC_CLOCKFREQMHZ)

// SyncIn params
#define CMT_PARAM_SYNCIN_MODE			0x00
#define CMT_PARAM_SYNCIN_SKIPFACTOR		0x01
#define CMT_PARAM_SYNCIN_OFFSET			0x02

// SyncIn mode
#define CMT_SYNCIN_DISABLED				0x0000
#define CMT_SYNCIN_EDGE_RISING			0x0001
#define CMT_SYNCIN_EDGE_FALLING			0x0002
#define CMT_SYNCIN_EDGE_BOTH			0x0003
#define CMT_SYNCIN_EDGE_MASK			0x0003
#define CMT_SYNCIN_TYPE_DOSAMPLING		0x0000
#define CMT_SYNCIN_TYPE_SENDLASTDATA	0x0004
#define CMT_SYNCIN_TYPE_CLOCK			0x0010
#define CMT_SYNCIN_TYPE_MASK			0x001C

// SyncOut params
#define CMT_PARAM_SYNCOUT_MODE			0x00
#define CMT_PARAM_SYNCOUT_SKIPFACTOR	0x01
#define CMT_PARAM_SYNCOUT_OFFSET		0x02
#define CMT_PARAM_SYNCOUT_PULSEWIDTH	0x03

// SyncOut mode
#define CMT_SYNCOUT_DISABLED		0x0000
#define CMT_SYNCOUT_TYPE_TOGGLE		0x0001
#define CMT_SYNCOUT_TYPE_PULSE		0x0002
#define CMT_SYNCOUT_POL_NEG			0x0000
#define CMT_SYNCOUT_POL_POS			0x0010
#define CMT_SYNCOUT_TYPE_MASK		0x000F
#define CMT_SYNCOUT_POL_MASK		0x0010

// OutputModes
#define CMT_OUTPUTMODE_MT9				0x8000
#define CMT_OUTPUTMODE_XM				0x0000
#define CMT_OUTPUTMODE_RAW				0x4000
//#define CMT_OUTPUTMODE_RAWGPSPRINT		0x1000	// Obsolete. Use CMT_OUTPUTMODE_GPSPVT_PRESSURE instead
#define CMT_OUTPUTMODE_GPSPVT_PRESSURE	0x1000
#define CMT_OUTPUTMODE_TEMP				0x0001
#define CMT_OUTPUTMODE_CALIB			0x0002
#define CMT_OUTPUTMODE_ORIENT			0x0004
#define CMT_OUTPUTMODE_AUXILIARY		0x0008
#define CMT_OUTPUTMODE_POSITION			0x0010
#define CMT_OUTPUTMODE_VELOCITY			0x0020
#define CMT_OUTPUTMODE_STATUS			0x0800

// CmtOutputSettings
#define CMT_OUTPUTSETTINGS_XM						0x00000001
#define CMT_OUTPUTSETTINGS_TIMESTAMP_NONE			0x00000000
#define CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT		0x00000001
#define CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLEUTC		0x00000002
#define CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION	0x00000000
#define CMT_OUTPUTSETTINGS_ORIENTMODE_EULER			0x00000004
#define CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX		0x00000008
#define CMT_OUTPUTSETTINGS_CALIBMODE_ACCGYRMAG		0x00000000
#define CMT_OUTPUTSETTINGS_CALIBMODE_ACC			0x00000060
#define CMT_OUTPUTSETTINGS_CALIBMODE_ACCGYR			0x00000040
#define CMT_OUTPUTSETTINGS_CALIBMODE_ACCMAG			0x00000020
#define CMT_OUTPUTSETTINGS_CALIBMODE_GYR			0x00000050
#define CMT_OUTPUTSETTINGS_CALIBMODE_GYRMAG			0x00000010
#define CMT_OUTPUTSETTINGS_CALIBMODE_MAG			0x00000030
#define CMT_OUTPUTSETTINGS_CALIBMODE_ACCGYRMAG_MASK	0x00000070
#define CMT_OUTPUTSETTINGS_DATAFORMAT_FLOAT			0x00000000
#define CMT_OUTPUTSETTINGS_DATAFORMAT_F1220			0x00000100
#define CMT_OUTPUTSETTINGS_DATAFORMAT_FP1632		0x00000200
#define CMT_OUTPUTSETTINGS_AUXILIARYMODE_AIN1		0x00000800			
#define CMT_OUTPUTSETTINGS_AUXILIARYMODE_AIN2		0x00000400
#define CMT_OUTPUTSETTINGS_POSITIONMODE_LLA_WGS84	0x00000000
#define CMT_OUTPUTSETTINGS_VELOCITYMODE_MS_XYZ		0x00000000
#define CMT_OUTPUTSETTINGS_TIMESTAMP_MASK			0x00000003
#define CMT_OUTPUTSETTINGS_ORIENTMODE_MASK			0x0000000C
#define CMT_OUTPUTSETTINGS_CALIBMODE_ACC_MASK		0x00000010
#define CMT_OUTPUTSETTINGS_CALIBMODE_GYR_MASK		0x00000020
#define CMT_OUTPUTSETTINGS_CALIBMODE_MAG_MASK		0x00000040
#define CMT_OUTPUTSETTINGS_CALIBMODE_MASK			0x00000070
#define CMT_OUTPUTSETTINGS_DATAFORMAT_MASK			0x00000300
#define CMT_OUTPUTSETTINGS_AUXILIARYMODE_AIN1_MASK	0x00000400
#define CMT_OUTPUTSETTINGS_AUXILIARYMODE_AIN2_MASK	0x00000800
#define CMT_OUTPUTSETTINGS_AUXILIARYMODE_MASK		0x00000C00
#define CMT_OUTPUTSETTINGS_POSITIONMODE_MASK		0x0001C000
#define CMT_OUTPUTSETTINGS_VELOCITYMODE_MASK		0x00060000
#define CMT_OUTPUTSETTINGS_COORDINATES_NED			0x80000000

// Extended (analog) Output Modes
#define CMT_EXTOUTPUTMODE_DISABLED			0x0000
#define CMT_EXTOUTPUTMODE_EULER				0x0001

// Processing flags
#define CMT_PROCESSINGFLAGS_IGBU_ENABLED	0x0001
#define CMT_PROCESSINGFLAGS_FIXED_GRAVITY	0x0002

// Send raw string mode
#define CMT_SENDRAWSTRING_INIT			0
#define CMT_SENDRAWSTRING_DEFAULT		1
#define CMT_SENDRAWSTRING_SEND			2

// Timeouts	
#define CMT_TO_DEFAULT					500
#define CMT_TO_INIT						250
#define CMT_TO_RETRY					50

#define	CMT_PERIOD_10HZ				11520		// invalid with gps pulse time correction
#define	CMT_PERIOD_12HZ				9600
#define	CMT_PERIOD_15HZ				7680		// invalid with gps pulse time correction
#define	CMT_PERIOD_16HZ				7200
#define	CMT_PERIOD_18HZ				6400		// invalid with gps pulse time correction
#define	CMT_PERIOD_20HZ				5760
#define	CMT_PERIOD_24HZ				4800
#define	CMT_PERIOD_25HZ				4608		// invalid with gps pulse time correction
#define	CMT_PERIOD_30HZ				3840
#define	CMT_PERIOD_32HZ				3600
#define	CMT_PERIOD_36HZ				3200
#define	CMT_PERIOD_40HZ				2880
#define	CMT_PERIOD_45HZ				2560		// invalid with gps pulse time correction
#define	CMT_PERIOD_48HZ				2400
#define	CMT_PERIOD_50HZ				2304		// invalid with gps pulse time correction
#define	CMT_PERIOD_60HZ				1920
#define	CMT_PERIOD_64HZ				1800
#define	CMT_PERIOD_72HZ				1600
#define	CMT_PERIOD_75HZ				1536		// invalid with gps pulse time correction
#define	CMT_PERIOD_80HZ				1440
#define	CMT_PERIOD_90HZ				1280		// invalid with gps pulse time correction
#define	CMT_PERIOD_96HZ				1200

#define	CMT_PERIOD_100HZ			1152
#define	CMT_PERIOD_120HZ			960
#define	CMT_PERIOD_128HZ			900
#define	CMT_PERIOD_144HZ			800
#define	CMT_PERIOD_150HZ			768			// invalid with gps pulse time correction
#define	CMT_PERIOD_160HZ			720
#define	CMT_PERIOD_180HZ			640
#define	CMT_PERIOD_192HZ			600
#define	CMT_PERIOD_200HZ			576
#define	CMT_PERIOD_225HZ			512			// invalid with gps pulse time correction
#define	CMT_PERIOD_240HZ			480
#define	CMT_PERIOD_256HZ			450
#define	CMT_PERIOD_288HZ			400
#define	CMT_PERIOD_300HZ			384
#define	CMT_PERIOD_320HZ			360
#define	CMT_PERIOD_360HZ			320
#define	CMT_PERIOD_384HZ			300
#define	CMT_PERIOD_400HZ			288
#define	CMT_PERIOD_450HZ			256			// invalid with gps pulse time correction
#define	CMT_PERIOD_480HZ			240
#define	CMT_PERIOD_512HZ			225

// openPort baudrates
#ifdef _WIN32
	#define CMT_BAUD_RATE_4800					CBR_4800
	#define CMT_BAUD_RATE_9600					CBR_9600
//	#define CMT_BAUD_RATE_14K4					CBR_14400
	#define CMT_BAUD_RATE_19K2					CBR_19200
//	#define CMT_BAUD_RATE_28K8					28800
	#define CMT_BAUD_RATE_38K4					CBR_38400
	#define CMT_BAUD_RATE_57K6					CBR_57600
	#define CMT_BAUD_RATE_115K2					CBR_115200
	#define CMT_BAUD_RATE_230K4					230400
	#define CMT_BAUD_RATE_460K8					460800
	#define CMT_BAUD_RATE_921K6					921600
#else
	#define CMT_BAUD_RATE_4800					B4800
	#define CMT_BAUD_RATE_9600					B9600
//	#define CMT_BAUD_RATE_14K4					B14400
	#define CMT_BAUD_RATE_19K2					B19200
//	#define CMT_BAUD_RATE_28K8					B28800
	#define CMT_BAUD_RATE_38K4					B38400
	#define CMT_BAUD_RATE_57K6					B57600
	#define CMT_BAUD_RATE_115K2					B115200
	#define CMT_BAUD_RATE_230K4					B230400
	#define CMT_BAUD_RATE_460K8					B460800
	#define CMT_BAUD_RATE_921K6					B921600
#endif

#define CMT_DEFAULT_OUTPUT_MODE			CMT_OUTPUTMODE_ORIENT
#define CMT_DEFAULT_OUTPUT_SETTINGS		(CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION |\
										 CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT)

#define CMT_GOTO_CONFIG_TRIES			3

#define CMT_MAX_DEVICES_PER_PORT		11
#define CMT_DEFAULT_SAMPLE_FREQUENCY	100
#define CMT_DEFAULT_PERIOD				1152
#define CMT_DEFAULT_SKIP				0
#define CMT_SYNCOUT_DEFAULT_PULSE_WIDTH	1000000		// 1ms = 1M ns

#define CMT_MAX_PORTS_PER_THREAD	20
#define CMT_MAX_FILES_PER_THREAD	20
#if CMT_MAX_PORTS_PER_THREAD >= CMT_MAX_FILES_PER_THREAD
#	define CMT_MAX_DEVICES			(CMT_MAX_PORTS_PER_THREAD * CMT_MAX_DEVICES_PER_PORT)
#else
#	define CMT_MAX_DEVICES			(CMT_MAX_FILES_PER_THREAD * CMT_MAX_DEVICES_PER_PORT)
#endif
#define CMT_MAX_PACKETS_PER_BUNDLE	CMT_MAX_PORTS_PER_THREAD
#define CMT_BUNDLE_QUEUE_SIZE		256
#define CMT_MAX_ITEMS_IN_PACKET		10

	//! The default size of the serial read buffer in bytes
#define CMT_DEFAULT_READ_BUFFER_SIZE		(CMT_MAXDATALEN*8)
	//! The default size of the serial write buffer in bytes
#define CMT_DEFAULT_WRITE_BUFFER_SIZE		CMT_MAXDATALEN
	//! The default baud rate of the Cmt1s serial communication
#define CMT_DEFAULT_BAUD_RATE				CMT_BAUD_RATE_115K2
	//! The size of the L2 data-message fifo-queue
#define CMT2_DATA_QUEUE_SIZE				60

	//! The timeout value for "goto config"-message acknowledgement
#define CMT3_CONFIG_TIMEOUT				100
	//! The default timeout value for blocking CMT1s operations in ms
#define CMT1_DEFAULT_TIMEOUT			10
	//! Timeout in ms for level 2
#define CMT2_DEFAULT_TIMEOUT			50
	//! The default timeout value for L3 data reading
#define CMT3_DEFAULT_TIMEOUT_MEAS		16
	//! The default timeout value for L3 configuration settings
#define CMT3_DEFAULT_TIMEOUT_CONF		3000
	//! The default timeout value for L4 data reading at L2 and 3
#define CMT4_DEFAULT_TIMEOUT_MEAS		1
	//! The default timeout value for L4 configuration settings
#define CMT4_DEFAULT_TIMEOUT_CONF		CMT3_DEFAULT_TIMEOUT_CONF
	//! The timeout to use for requests during measurement mode
#define CMT4_MEASUREMENT_REQ_TIMEOUT	200
	//! The standard timeout to use for data receipt in measurement mode
#define CMT4_DEFAULT_TIMEOUT_DATA		3000
	//! The timeout used for reading a wakeup message in config mode
#define CMT4_CONFIGWAIT_TIMEOUT			50

enum CmtControlLine {
	CMT_CONTROL_DCD		= 0x0001,		// pin 1: Carrier Detect
	CMT_CONTROL_RD		= 0x0002,		// pin 2: Received Data
	CMT_CONTROL_TD		= 0x0004,		// pin 3: Transmitted Data
	CMT_CONTROL_DTR		= 0x0008,		// pin 4: Data Terminal Ready
	CMT_CONTROL_GND		= 0x0010,		// pin 5: Common Ground
	CMT_CONTROL_DSR		= 0x0020,		// pin 6: Data Set Ready
	CMT_CONTROL_RTS		= 0x0040,		// pin 7: Request To Send
	CMT_CONTROL_CTS		= 0x0080,		// pin 8: Clear To Send
	CMT_CONTROL_RI		= 0x0100		// pin 9: Ring Indicator
};

// Reset orientation message type
enum CmtResetMethod {
	CMT_RESETORIENTATION_STORE = 0,
	CMT_RESETORIENTATION_HEADING,
	CMT_RESETORIENTATION_GLOBAL,
	CMT_RESETORIENTATION_OBJECT,
	CMT_RESETORIENTATION_ALIGN
};

enum CmtXmSyncMode {
	CMT_XM_SYNC_OFF				= 0x00,
	CMT_XM_SYNC_PWM				= 0x01,
	CMT_XM_SYNC_MASTER			= 0x10,
	CMT_XM_SYNC_SLAVE			= 0x20,
	CMT_XM_SYNC_TOGGLE			= 0x30
};

//! The type of a Device Id
typedef uint32_t	CmtDeviceId;

//! A structure for storing the firmware version
struct CmtVersion {
	uint8_t m_major;
	uint8_t m_minor;
	uint8_t m_revision;
};

/* different alignment commands for gcc / MSVS, the structure needs to be 2-byte aligned
	because the deviceId field is 4 bytes long on offset 98.
*/
#ifdef _MSC_VER
	#pragma pack(push, 2)
#endif
//! Structure containing a full device configuration as returned by the ReqConfig message.
struct CmtDeviceConfiguration {
	uint32_t	m_masterDeviceId;
	uint16_t	m_samplingPeriod;
	uint16_t	m_outputSkipFactor;
	uint16_t	m_syncinMode;
	uint16_t	m_syncinSkipFactor;
	uint32_t	m_syncinOffset;
	uint8_t	m_date[8];
	uint8_t	m_time[8];
	uint8_t	m_reservedForHost[32];
	uint8_t	m_reservedForClient[32];
	uint16_t	m_numberOfDevices;
	struct _devInfo {
		uint32_t	m_deviceId;
		uint16_t	m_dataLength;
		uint16_t	m_outputMode;
		uint32_t	m_outputSettings;
		uint16_t	m_currentScenario;
		uint8_t		m_fwRevMajor;
		uint8_t		m_fwRevMinor;
		uint8_t		m_fwRevRevision;
		uint8_t		m_filterType;
		uint8_t		m_filterMajor;
		uint8_t		m_filterMinor;
	} m_deviceInfo[CMT_MAX_DEVICES_PER_PORT];

#if !defined(CMT_DLL_IMPORT)
	void readFromMessage(const void* message);
#endif
}
// different alignment commands for gcc / MSVS
#ifdef _MSC_VER
	;
	#pragma pack(pop)
#else
	/*! \cond NODOXYGEN */	__attribute__((__packed__))	/*! \endcond */;
#endif

//////////////////////////////////////////////////////////////////////////////////////////

	//! An output mode bit-field
typedef uint32_t CmtOutputMode;
	//! An output settings bit-field
typedef uint64_t CmtOutputSettings;

	//! A structure for storing data formats
struct CmtDataFormat {
	CmtOutputMode		m_outputMode;
	CmtOutputSettings	m_outputSettings;

#if !defined(CMT_DLL_IMPORT)
	//! default constructor, initializes to the given (default) MT settings
	CmtDataFormat(	const CmtOutputMode mode = CMT_DEFAULT_OUTPUT_MODE,
					const CmtOutputSettings settings = CMT_DEFAULT_OUTPUT_SETTINGS)
					: m_outputMode(mode), m_outputSettings(settings) {}
#endif
};

	//! An MT timestamp (sample count)
typedef uint16_t CmtMtTimeStamp;

#define CMT_MAX_FILENAME_LENGTH	512

#define CMT_DID_BROADCAST				0x80000000
#define CMT_DID_MASTER					0

#ifndef NOT_FOR_PUBLIC_RELEASE
#ifdef _CMT_ADD_PRIVATE
#	include "xsens_private.h"
#endif
#endif

//! A structure for storing sync in settings
struct CmtSyncInSettings {
	uint16_t	m_mode;
	uint16_t	m_skipFactor;
	uint32_t	m_offset;		//!< Offset in ns

#if !defined(CMT_DLL_IMPORT)
	//! default constructor, initializes to the given (default) MT settings
	CmtSyncInSettings(	const uint16_t mode = 0,
						const uint16_t skip = 0,
						const uint32_t offset = 0)
						: m_mode(mode), m_skipFactor(skip), m_offset(offset) {}
#endif
};

//! A structure for storing sync out settings
struct CmtSyncOutSettings {
	uint16_t	m_mode;
	uint16_t	m_skipFactor;
	uint32_t	m_offset;		//!< Offset in ns
	uint32_t	m_pulseWidth;	//!< Pulse width in ns

#if !defined(CMT_DLL_IMPORT)
	//! default constructor, initializes to the given (default) MT settings
	CmtSyncOutSettings( const uint16_t mode = 0,
						const uint16_t skip = 0,
						const uint32_t offset = 0,
						const uint32_t width = CMT_SYNCOUT_DEFAULT_PULSE_WIDTH) :
						m_mode(mode), m_skipFactor(skip), m_offset(offset),
						m_pulseWidth(width) {}
#endif
};

//! A structure for storing UTC Time values
struct CmtUtcTime
{
	uint32_t	m_nano;
	uint16_t	m_year;
	uint8_t	m_month;
	uint8_t	m_day;
	uint8_t	m_hour;
	uint8_t	m_minute;
	uint8_t	m_second;
	uint8_t	m_valid;	//!< When set to 1, the time is valid, when set to 2, the time part is valid, but the date may not be valid. when set to 0, the time is invalid and should be ignored.
};

//! A structure for storing device modes
struct CmtDeviceMode {
	CmtOutputMode		m_outputMode;
	CmtOutputSettings	m_outputSettings;
	uint16_t		m_sampleFrequency;

#if !defined(CMT_DLL_IMPORT)
	//! default constructor, initializes to the given (default) MT settings
	CmtDeviceMode(	const CmtOutputMode mode = CMT_DEFAULT_OUTPUT_MODE,
					const CmtOutputSettings settings = CMT_DEFAULT_OUTPUT_SETTINGS,
					const uint16_t frequency = CMT_DEFAULT_SAMPLE_FREQUENCY) :
						m_outputMode(mode), m_outputSettings(settings),
						m_sampleFrequency(frequency) {}
	
	/*! \brief Compute the period and skip factor.
	
		This function computes the period and skipFactor fields from the stored 
		m_sampleFrequency field. The maximum error in the frequency is approximately 
		0.4%, which occurs at 510Hz (= actually 512 Hz). In general, the higher 
		frequencies are harder to set up exactly.
	*/
	void getPeriodAndSkipFactor(uint16_t& period, uint16_t& skip) const;
	/*! \brief Return the real sample frequency in Hz. 
	
		This may be up to 2Hz different from the value that is set.
	*/
	double getRealSampleFrequency(void) const;
	/*! \brief Compute the sample frequency from a period and skip factor.
	
		This function does the reverse of the getPeriodAndSkipFactor function, storing the
		value in the m_sampleFrequency field.
	*/
	void setPeriodAndSkipFactor(uint16_t period, uint16_t skip);

	//! Check if all fields of the two structures are equal
	bool operator == (const CmtDeviceMode& dev) const;
#endif
};

//! A structure for storing device modes using period and skip factor (new default)
struct CmtDeviceMode2 {
	CmtOutputMode		m_outputMode;
	CmtOutputSettings	m_outputSettings;
	uint16_t		m_period;
	uint16_t		m_skip;

#if !defined(CMT_DLL_IMPORT)
	//! default constructor, initializes to the given (default) MT settings
	CmtDeviceMode2(	const CmtOutputMode mode = CMT_DEFAULT_OUTPUT_MODE,
		const CmtOutputSettings settings = CMT_DEFAULT_OUTPUT_SETTINGS,
		const uint16_t period = CMT_DEFAULT_PERIOD,
		const uint16_t skip = CMT_DEFAULT_SKIP) :
	m_outputMode(mode), m_outputSettings(settings),
		m_period(period), m_skip(skip) {}

	/*! \brief Return the real sample frequency in Hz. 

	This may be up to 2Hz different from the value that is set.
	*/
	double getRealSampleFrequency(void) const;
	/*! \brief Return the sample frequency in Hz. 

	This may be up to 2Hz different from the value that is set.
	*/
	uint16_t getSampleFrequency(void) const;
	/*! \brief Compute the period and skip factor from a sample frequency.

	This function does the reverse of the getPeriodAndSkipFactor function, storing the
	value in the m_period and m_skip field.
	*/
	void setSampleFrequency(uint16_t freq);

	//! Check if all fields of the two structures are equal
	bool operator == (const CmtDeviceMode2& dev) const;
#endif
};

#define CMT_MAX_SCENARIOS_IN_MT		5
#define CMT_MAX_SCENARIOS			254

//! A structure for storing scenario information
struct CmtScenario {
	uint8_t m_type;		//!< The type of the scenario. When set to 255 in an operation, the 'current' scenario is used.
	uint8_t m_version;	//!< The version of the scenario.
	char m_label[CMT_LEN_SCENARIOLABEL+1];		//!< The label of the scenario.
	char m_filterType;							//!< The type of the XKF filter this scenario is intended for '3': XKF-3, '6': XKF-6. \note The value is a character, so XKF-3 is '3', which is hex 0x33
	uint8_t m_filterMajor, m_filterMinor;		//!< The version of the XKF filter this scenario is intended for
};

enum CmtCallbackType {
	CMT_CALLBACK_LIST_TERMINATOR	= -1,//!< Not a callback function. Used to define the end of a CallbackSelector list

	CMT_CALLBACK_ONMEASUREMENTPREPARE,	//!< Called right before sending a GotoMeasurement message
	CMT_CALLBACK_ONMEASUREMENTSTART,	//!< Called right after successfully switching to Measurement mode
	CMT_CALLBACK_ONMEASUREMENTSTOP,		//!< Called right before switching from Measurement mode to Config mode
	CMT_CALLBACK_ONPOSTPROCESS,			//!< Called when a full data bundle is available and has been processed by the CMT. The first void* parameter supplied to this function can be handed as the bundle parameter in cmtData... The function is intended to manipulate or process the newly received bundle. Manipulation is allowed, but only 'custom data' will be written to a file when logging.
	CMT_CALLBACK_ONBYTESRECEIVED,		//!< Called when bytes have been read from a port
	CMT_CALLBACK_ONMESSAGERECEIVED,		//!< Called when a full message has been received from a port
	CMT_CALLBACK_ONMESSAGESENT,			//!< Called when a full message has been sent by a port
	CMT_CALLBACK_ONMISSINGSAMPLES,		//!< Called when interpolation mode != 0 and missing samples are detected
	CMT_CALLBACK_ONWAKEUPRECEIVED,		//!< Called when receiving a wakeup message in config mode. Advise to refresh that port cache as soon as possible
	CMT_CALLBACK_ONINSTANCE,			//!< Called when a new CMT instance was created or an existing one is about to be destroyed. Also called once for all existing CMT instances when a plugin becomes enabled. If param is non-null, the instance is created, otherwise it is about to be destroyed. The return value of this callback is ignored.
	CMT_CALLBACK_ONPLAYBACK,				//!< Called when a full data bundle is available and has been processed by the CMT. The first void* parameter supplied to this function can be handed as the bundle parameter in cmtData... The function is intended to process the newly read bundle. Manipulation is allowed, but nothing gets written back to the file.

	CMT_CALLBACK_COUNT					//!< Not a callback function. Used internally for list management
};

#define CmtCallbackSelector	CmtCallbackType

enum CmtQueueMode {
	CMT_QM_FIFO	= 0,
	CMT_QM_LAST = 1,
	CMT_QM_RAW  = 2
};

struct CmtBinaryData {
	int32_t m_size;
	uint8_t m_data[CMT_MAXMSGLEN];
	uint16_t m_portNr;
};

//! \brief Function prototype for Cmt callback functions
typedef void (__cdecl * CmtCallbackFunction)(int32_t instance, int32_t callbackType, void* param, void* userParam);

//! \brief Structure for storing information about a serial port
struct CmtPortInfo {
	uint32_t m_baudrate;	//!< The baudrate at which an Xsens device was detected
	uint32_t m_deviceId;	//!< The device Id of the detected Xsens device
	uint16_t m_portNr;		//!< The port number
	char m_portName[256];		//!< The port name
#ifdef __cplusplus
#	ifdef _WIN32
		//! greater than operator, used for sorting the list
	bool operator > (const CmtPortInfo& p) const { return m_portNr > p.m_portNr; }
		//! less than operator, used for sorting the list
	bool operator < (const CmtPortInfo& p) const { return m_portNr < p.m_portNr; }
		//! equality operator, used for finding items in a list
	bool operator == (const CmtPortInfo& p) const { return m_portNr == p.m_portNr; }
		//! equality operator, used for finding items in a list
	bool operator == (const uint16_t port) const { return m_portNr == port; }
		//! equality operator, used for finding items in a list
	bool operator == (const char *port) const { return strcmp(m_portName, port) == 0; }
#	else	// Linux
		//! greater than operator, used for sorting the list
	bool operator > (const CmtPortInfo& p) const { return strcmp(m_portName, p.m_portName) > 0; }
		//! less than operator, used for sorting the list
	bool operator < (const CmtPortInfo& p) const { return strcmp(m_portName, p.m_portName) < 0; }
		//! equality operator, used for finding items in a list
	bool operator == (const CmtPortInfo& p) const { return strcmp(m_portName, p.m_portName) == 0; }
		//! equality operator, used for finding items in a list
	bool operator == (const uint16_t port) const { (void)port; return false; }
		//! equality operator, used for finding items in a list
	bool operator == (const char *port) const { return strcmp(m_portName, port) == 0; }
#	endif // ifdef _WIN32/Linux
#endif
};

struct CmtShortVector {
	uint16_t m_data[3];
};
struct CmtRawData {
	CmtShortVector	m_acc,m_gyr,m_mag;
	uint16_t	m_temp;
};
struct CmtGpsPvtData {	uint16_t	m_pressure;
	uint8_t		m_pressureAge;
	uint32_t	m_itow;
	int32_t		m_latitude;
	int32_t		m_longitude;
	int32_t		m_height;
	int32_t		m_veln;
	int32_t		m_vele;
	int32_t		m_veld;
	uint32_t	m_hacc;
	uint32_t	m_vacc;
	uint32_t	m_sacc;
	uint8_t	m_gpsAge;
};
#define CmtRawGpsData	CmtGpsPvtData		//!< obsolete, use CmtGpsPvtData instead
struct CmtRawPressureData {
	uint16_t	m_pressure;
	uint8_t		m_pressureAge;
};
struct CmtAnalogInData
{
	uint16_t m_data;
};
struct CmtVector {
	double m_data[3];
};
struct CmtCalData {
	CmtVector	m_acc,m_gyr,m_mag;
};
struct CmtQuat {
	double m_data[4];
};
struct CmtEuler {
	double m_roll;		//!< The roll (rotation around x-axis / back-front-line)
	double m_pitch;		//!< The pitch (rotation around y-axis / right-left-line)
	double m_yaw;		//!< The yaw (rotation around z-axis / down-up-line)
};
struct CmtMatrix {
	double m_data[3][3];
};

#define CMT_MAX_SVINFO		16
struct CmtGpsSatelliteInfo {
	uint8_t m_id;
	uint8_t m_navigationStatus;
	uint8_t m_signalQuality;
	uint8_t m_signalStrength;
};
struct CmtGpsStatus {
	CmtGpsSatelliteInfo m_svInfo[CMT_MAX_SVINFO];
};

typedef int64_t CmtTimeStamp;

#define CMT_AUTO_SAVE_FRAMES	5000
#define CMT_FILE_LAST_FRAME		0xFFFFFFFF

#define CMT_BID_BROADCAST		(const uint8_t)0x00
#define CMT_BID_INVALID			(const uint8_t)0xFE
#define CMT_MID_REQEMTS			(const uint8_t)0x90
#define CMT_MID_EMTSDATA		(const uint8_t)0x91
#define CMT_EMTS_SIZE			1320

#define CMT_STATUSFLAG_SELFTEST_OK					0x01
#define CMT_STATUSFLAG_XKFVALID						0x02
#define CMT_STATUSFLAG_GPSVALID						0x04
#define CMT_STATUSFLAG_NOROTATION_MASK				0x18
#define CMT_STATUSFLAG_NOROTATION 					0x18
#define CMT_STATUSFLAG_NOROTATION_ABORTED 			0x10
#define CMT_STATUSFLAG_NOROTATION_SAMPLES_REJECTED 	0x08

#define CMT_MAX_VPORTNAME_LEN	32

#endif
/*! \file
	\brief CMT library main header file.

	This file is intended for people that want to use the Xsens CMT library.
	This file was automatically generated, so do not judge it based on its layout or
	redundant definitions.

	\section FileCopyright Copyright Notice
	Copyright (C) Xsens Technologies B.V., 2006.  All rights reserved.

	This source code is intended for use only by Xsens Technologies BV and
	those that have explicit written permission to use it from
	Xsens Technologies BV.

	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
	PARTICULAR PURPOSE.
*/

#ifndef CMTDLL_H
#define CMTDLL_H

#ifdef CMT_DLL_EXPORT
//#    include "cmtdll_pre.h"		// no longer required by dll/com build
#    include "xsens_std.h"
#    include "cmtdef.h"
#endif

CMT_DLL_API XsensResultValue cmtAbortPortScan(void);					//!< \brief Abort any port scan currently in progress. \details This function can be used to abort a previous cmtScanPort or cmtScanPorts call. Since cmtScanPort and cmtScanPorts are blocking calls, this function has to be called from another thread. The scan will stop as soon as possible, but not immediately, since the current scan (a single port at a single baud rate) will be completed before aborting. The scan function will return XRV_ERROR when aborted in this way.
CMT_DLL_API int32_t cmtCreateInstance(const char* serialNumber);			//!< \brief Create a new CMT instance. \details When using the CMT.dll, an instance number is required for most function calls. This instance identifies the distinct CMT object that the function should be perfomed on. This function supplies that number after creating the new CMT object and returns the created instance number. If the function fails for any reason, -1 is returned. \param serialNumber should contain a valid serial number as supplied upon purchase of the SDK, including the dashes.
CMT_DLL_API XsensResultValue cmtDestroyInstance(const int32_t instance);	//!< \brief Destroy a CMT instance. \details This function destroys the given CMT object, freeing up its instance number for use by another process. It will close all files and ports associated with the instance. This function requires a valid instance number.\n If -1 is supplied for the instance nr, all creatd instances will be destroyed. For proper termination, it is strongly advised to destroy all instances before the calling process terminates.
CMT_DLL_API XsensResultValue cmtClearLastHwError(const int32_t instance);	//!< \brief Clear the 'last error' value. \details This function can be used to tell CMT that the last HW error has been handled and should be cleared.
CMT_DLL_API XsensResultValue cmtClose (const int32_t instance);											//!< \brief Close all devices connected to the given instance. \details This function closes all files and ports associated with the supplied instance.
CMT_DLL_API XsensResultValue cmtCloseLogFile(const int32_t instance, const char* filename);				//!< \brief Close an open log file. \details This function closes a logfile which is associated with the supplied instance. \param filename The name of the file as supplied to cmtCreateLogFile or cmtOpenLogFile.
CMT_DLL_API XsensResultValue cmtCloseLogFileW(const int32_t instance, const wchar_t* filename);			//!< \brief Close an open log file. \details This function closes a logfile which is associated with the supplied instance. \param filename The name of the file as supplied to cmtCreateLogFile or cmtOpenLogFile.
CMT_DLL_API XsensResultValue cmtCloseLogFileForPort (const int32_t instance, const uint16_t portNr);	//!< \brief Close an open log file by associated port number. \details This function closes an open log file that is associated with the given port. \param portNr The port number for which the log file should be closed.
#ifndef NOT_FOR_PUBLIC_RELEASE
#ifdef CMT_TESTBENCH
class SerialisedMLStruct;
CMT_DLL_API XsensResultValue cmtFastFloatToDouble(const int32_t instance, double *dbl, int16_t exponent, int32_t mantissa); //!< \brief Convert a FastFloat to a double.
CMT_DLL_API XsensResultValue cmtDoubleToFastFloat(const int32_t instance, int16_t *exponent, int32_t *mantissa, double dbl); //!< \brief Convert a double to a FastFloat.
CMT_DLL_API XsensResultValue cmtGetRawEmts (const int32_t instance, SerialisedMLStruct **rawEmts, const CmtDeviceId deviceId);	//!< \brief Get the decrypted eMTS byte string from a device.
CMT_DLL_API XsensResultValue getEmtsStructureVersion (const int32_t instance, SerialisedMLStruct **emtsStructureVersion, const CmtDeviceId deviceId); //!< \brief Get the version of the eMTS structure.
CMT_DLL_API XsensResultValue cmtGetRawConfiguration (const int32_t instance, SerialisedMLStruct **configuration, const CmtDeviceId deviceId); //!< \brief Get the raw configuration data from a device.
CMT_DLL_API XsensResultValue cmtGetSettings (const int32_t instance, SerialisedMLStruct **settings, const CmtDeviceId deviceId); //!< \brief Get all settings from the loaded settings file.
CMT_DLL_API XsensResultValue cmtLoadScenariosOverride(const int32_t instance, const char* filename, const char *const *const overrideFilename); //!< \brief Load scenario definitions from a file. \details Use this function to read the scenario definitions from the given (xsb) file. The override files are xsa files whose settings take precedence over the corresponding settings in the xsb file.
CMT_DLL_API XsensResultValue cmtLoadScenariosOverrideW(const int32_t instance, const wchar_t* filename, const wchar_t *const *const overrideFilename); //!< \brief Load scenario definitions from a file. \details Use this function to read the scenario definitions from the given (xsb) file. The override files are xsa files whose settings take precedence over the corresponding settings in the xsb file.
CMT_DLL_API XsensResultValue cmtDataGetDebugInfo(const int32_t instance, SerialisedMLStruct ** debugInfo, const CmtDeviceId dev, void* bundle); //!< \brief Get debug info. \details This function extracts the XKF debug info from a packet.
CMT_DLL_API XsensResultValue cmtGetReplayMode(const int32_t instance, uint8_t* mode,	const CmtDeviceId deviceId); //!< \brief Get replay mode. \details This function gets the current replay mode.
CMT_DLL_API XsensResultValue cmtSetReplayMode(const int32_t instance, uint8_t mode,	const CmtDeviceId deviceId); //!< \brief Set replay mode. \details This function sets the current replay mode.
CMT_DLL_API XsensResultValue cmtSetReplayCalData(const int32_t instance, double *calData, CmtDeviceId deviceId); //!< \brief Set calibrated data for replay mode. \details This function sets up the calibrated data for the current replay mode.
CMT_DLL_API XsensResultValue cmtInitXkfKic(const int32_t instance, void *input); //!< \brief Initialises XKF-KiC-MVN. \details This function initialises XKF-KiC-MVN.
CMT_DLL_API XsensResultValue cmtOneStepXkfKic(const int32_t instance, void *input, void *output, void *debug); //!< \brief Single iteration of XKF-KiC-MVN. \details This function runs a single iteration of XKF-KiC-MVN.
CMT_DLL_API XsensResultValue cmtTestXkfKic(const int32_t instance, const char *testSettingsFileName); //!< \brief run scripted Kic tests
CMT_DLL_API XsensResultValue cmtTestXkf3(const int32_t instance, const char *testSettingsFileName); //!< \brief run scripted XKF-3 tests
CMT_DLL_API XsensResultValue cmtTestXkf6(const int32_t instance, const char *testSettingsFileName); //!< \brief run scripted XKF-6 tests
#endif
#endif
#ifdef _WIN32
CMT_DLL_API XsensResultValue cmtClosePort (const int32_t instance, const uint16_t portNr);								//!< \brief Close a serial communication port. \details This function closes a specific serial port. \param portNr The number of the port to open. eg when opening COM1, supply a 1
#else
CMT_DLL_API XsensResultValue cmtClosePort (const int32_t instance, const char *portName);
#endif
#ifdef _WIN32
CMT_DLL_API XsensResultValue cmtCreateLogFile(const int32_t instance, const uint16_t portNr, const char* filename);		//!< \brief Create a log file for the given port. \details This function creates a log file for the given port. CMT must be connected to the port or the function will fail. \param portNr The number of the port for which a log file should be created. \param filename The name of the file to use as a log file. A fully qualified (local) path+filename is recommended. \remarks If a writable file of the same name already exists in the target location, it will be overwritten without warning.
CMT_DLL_API XsensResultValue cmtCreateLogFile2(const int32_t instance, const char *portName, const char* filename);
#else
CMT_DLL_API XsensResultValue cmtCreateLogFile(const int32_t instance, const char *portName, const char* filename);
#endif
#ifdef _WIN32
CMT_DLL_API XsensResultValue cmtCreateLogFileW(const int32_t instance, const uint16_t portNr, const wchar_t* filename);		//!< \brief Create a log file for the given port. \details This function creates a log file for the given port. CMT must be connected to the port or the function will fail. \param portNr The number of the port for which a log file should be created. \param filename The name of the file to use as a log file. A fully qualified (local) path+filename is recommended. \remarks If a writable file of the same name already exists in the target location, it will be overwritten without warning.
CMT_DLL_API XsensResultValue cmtCreateLogFileW2(const int32_t instance, const char *portName, const wchar_t *filename);
#else
CMT_DLL_API XsensResultValue cmtCreateLogFileW(const int32_t instance, const char *portName, const wchar_t *filename);
#endif

CMT_DLL_API XsensResultValue cmtFlushDataBuffer(const int32_t instance, const int32_t complete);									//!< \brief Flush all incoming data from the inbound data queues. \details This function flushes any remaining data from the inbound data queue, effectively clearing it of all unprocessed data. \param complete When this boolean value is set to non-zero, the internal working queue is flushed along with the ready queue. This is not recommended, but necessary in some extraordinary cases. \see cmtGetQueueLength

CMT_DLL_API XsensResultValue cmtForceSetDeviceMode(const int32_t instance, const CmtOutputMode mode, const CmtOutputSettings settings, const uint16_t frequency, const CmtDeviceId deviceId);	//!< \brief Set the complete output mode of a device. \details This function updates the complete output mode of the specified device. In contrast with cmtSetDeviceMode,this function always sets all values, even when the device already reports the same setting. When setting the sample frequency of a device connected to an Xbus Master, the sample frequency of the Xbus Master is updated instead. \see cmtGetDeviceMode cmtSetDeviceMode
CMT_DLL_API XsensResultValue cmtForceSetDeviceMode2(const int32_t instance, const CmtOutputMode mode, const CmtOutputSettings settings, const uint16_t period, const uint16_t skipFactor, const CmtDeviceId deviceId);	//!< \brief Set the complete output mode of a device. \details This function updates the complete output mode of the specified device. In contrast with cmtSetDeviceMode,this function always sets all values, even when the device already reports the same setting. When setting the sample frequency of a device connected to an Xbus Master, the sample frequency of the Xbus Master is updated instead. \see cmtGetDeviceMode cmtSetDeviceMode

CMT_DLL_API XsensResultValue cmtGetAvailableScenarios(const int32_t instance, CmtScenario* scenarios, const CmtDeviceId deviceId);			//!< \brief Retrieve a list of the available scenarios. \details Use this function to retrieve a list of all scenarios available in a Motion Tracker. See the supplied documentation for more information about scenarios. \param scenarios A buffer for storing the available scenarios. The supplied buffer should be able to contain at least 6 scenarios, but for future devices a larger buffer is advised. Use CMT_MAX_SCENARIOS_IN_MT+1. The list is terminated by a scenario that has type 0. \remarks The type of any of the returned scenarios can be safely supplied to the cmtSetScenario function. \see cmtGetScenario cmtSetScenario cmtGetAvailableSoftwareScenarios cmtGetSoftwareScenario cmtSetSoftwareScenario
CMT_DLL_API XsensResultValue cmtGetAvailableSoftwareScenarios(const int32_t instance, CmtScenario* scenarios, const CmtDeviceId deviceId);	//!< \brief Retrieve a list of the available software scenarios. \details Use this function to retrieve a list of all scenarios available for software processing for a Motion Tracker. The available scenarios should be loaded first with a call to cmtLoadScenarios. See the supplied documentation for more information about scenarios. \param scenarios A buffer for storing the available scenarios. The supplied buffer should be able to contain at least 256 scenarios. Use CMT_MAX_SCENARIOS+1. The list is terminated by a scenario that has type 0. \remarks The type of any of the returned scenarios can be safely supplied to the cmtSetSoftwareScenario and cmtUploadScenario functions. \see cmtGetScenario cmtSetScenario cmtGetAvailableScenarios cmtGetSoftwareScenario cmtSetSoftwareScenario cmtUploadScenario cmtLoadScenarios
CMT_DLL_API XsensResultValue cmtGetBatteryLevel(const int32_t instance, uint8_t* level, const CmtDeviceId deviceId);					//!< \brief Get an Xbus Master's battery level. \details The battery level is a value between 0 and 255 that indicates the voltage of the batteries. The scale is not linear and the values should not be used as an absolute voltage. The amount of time remaining for measurement given any battery level greatly depends on the type of batteries used, the number of sensors attached to the Xbus Master and the used output options.
CMT_DLL_API XsensResultValue cmtGetBaudrate (const int32_t instance, uint32_t* baudrate, const CmtDeviceId deviceId);					//!< \brief Get the baudrate used by a port. \details This function returns the baud rate at which the port is currently connected. The function will fail when no port is connected.
CMT_DLL_API XsensResultValue cmtGetBluetoothState(const int32_t instance, int32_t* enabled, const CmtDeviceId deviceId);						//!< \brief Get the state of the bluetooth communication. \details This function tells whether the Bluetooth connection of the Xbus Master is on (1) or off (0).
CMT_DLL_API XsensResultValue cmtGetBusPowerState(const int32_t instance, int32_t* enabled, const CmtDeviceId deviceId);							//!< \brief Get the state of the Xbus power. \details This function tells whether the Xbus of the supplied Xbus Master is currently switched on (1) or not (0). When it is switched off, the attached MT devices have no power and communication with them is not possible. Before going to measurement mode, use cmtSetBusPowerState to restore power. \see cmtSetBusPowerState
CMT_DLL_API XsensResultValue cmtGetConfiguration (const int32_t instance, CmtDeviceConfiguration* configuration, const CmtDeviceId deviceId);	//!< \brief Get device configuration. \details This function retrieves the complete device configuration of a single device.
CMT_DLL_API XsensResultValue cmtGetDataLength (const int32_t instance, uint32_t* length, const CmtDeviceId deviceId);						//!< \brief Retrieve data size. \details This function retrieves the number of bytes that are in a data message as sent by the given device.
CMT_DLL_API XsensResultValue cmtGetDeviceCount (const int32_t instance, uint16_t* count);												//!< \brief Retrieve total device count. \details This function retrieves the total number of connected (master + slave) devices or 0 if not connected. \see cmtGetMtCount cmtGetMainDeviceCount
CMT_DLL_API XsensResultValue cmtGetDeviceId(const int32_t instance, CmtDeviceId* deviceId, const uint16_t index);						//!< \brief Retrieve the DeviceId of a device given its index. \details This function retrieves the DeviceId for the device at the given index. A valid index is in the range of 0 to the value supplied by cmtGetDeviceCount. When no devices are connected, a 0 ID is supplied. \remarks When a device is connected or disconnected, the list of devices will be sorted again. This may change the index of any or all devices in the list. \see cmtGetMtDeviceId cmtGetXmDeviceId
CMT_DLL_API XsensResultValue cmtGetDeviceIdFromLocationId(const int32_t instance, CmtDeviceId* deviceId, const uint16_t locationId);		//!< \brief Retrieve a DeviceId based on a locationId. \details This function will search all connected devices for the requested locationId. If a device with a matching locationId is found, its DeviceId is supplied. Otherwise, a 0 ID is supplied. \param deviceId The resulting DeviceID. \param locationId The requested location ID
/*! \brief Get the complete output mode of a device. 
\details This function reads the complete output mode of the specified device. 
When reading the sample frequency of a device connected to an Xbus Master, the sample frequency of the Xbus Master is returned instead.

\param mode The mode of the device. Output mode values can be obtained through the OUTPUTMODE_* functions.
\param settings Mode specific settings for the device. The settings values can be obtained through the OUTPUTSETTINGS_* functions.
\param frequency The output frequency of the device in Hz. 
\param deviceId The ID of the device that has to be updated.

\see cmtSetDeviceMode cmtGetDeviceMode2 cmtSetDeviceMode2
*/
CMT_DLL_API XsensResultValue cmtGetDeviceMode (const int32_t instance, CmtDeviceMode* mode, const CmtDeviceId deviceId);
/*! \brief Get the complete output mode2 of a device. 
\details This function reads the complete output mode of the specified device.
When reading the sample frequency of a device connected to an Xbus Master, the sample frequency of the Xbus Master is returned instead.

\param mode The mode of the device. Output mode values can be obtained through the OUTPUTMODE_* functions.
\param settings Mode specific settings for the device. The settings values can be obtained through the OUTPUTSETTINGS_* functions.
\param period The sample period for the device. The period values can be obtained through the PERIOD_* functions.
\param skipFactor The output skip factor for the device. 
\param deviceId The ID of the device that has to be updated.

\see cmtSetDeviceMode2 cmtGetDeviceMode cmtSetDeviceMode
*/
CMT_DLL_API XsensResultValue cmtGetDeviceMode2 (const int32_t instance, CmtDeviceMode2* mode, const CmtDeviceId deviceId);
CMT_DLL_API XsensResultValue cmtGetDllVersion(CmtVersion* version);																//!< \brief Get the version of the DLL. \details This function returns the complete version of the DLL in a CmtVersion structure.
CMT_DLL_API XsensResultValue cmtGetDroppedPacketCount(const int32_t instance, uint32_t* count, const CmtDeviceId deviceId);	//!< \brief Retrieve number of dropped packets. \details The function will return the number of packets dropped by the port, for a given device. Packets can be dropped for several reasons, including checksum failures and sample-counter mismatch in synchronized operation.
CMT_DLL_API XsensResultValue cmtGetErrorMode(const int32_t instance, uint16_t* mode, const CmtDeviceId deviceId);			//!< \brief Return the error mode of the device. \details This function returns the error mode of the device. The error mode determines how the device handles errors. See the low-level communication documentation for more details. \see cmtSetErrorMode
CMT_DLL_API XsensResultValue cmtGetFirmwareRevision (const int32_t instance, CmtVersion* revision, const CmtDeviceId deviceId);	//!< \brief Return Firmware revision. \details This function retrieves the firmware version that is currently installed in the device.
CMT_DLL_API XsensResultValue cmtGetGpsLeverArm(const int32_t instance, CmtVector* arm, const CmtDeviceId deviceId);				//!< \brief Get the currently used GPS lever arm. \details Use this function to retrieve the vector currently used as the GPS lever arm in meters. The GPS lever arm is the relative position of the GPS antenna to the MTi-G unit. The arm is specified in the object coordinate system. See the manual for more information on coordinate systems, alignment resets and the lever arm. \see cmtSetGpsLeverArm
CMT_DLL_API XsensResultValue cmtGetGpsStatus(const int32_t instance, CmtGpsStatus* status, const CmtDeviceId deviceId);			//!< \brief Request the status of the GPS satellites in orbit. \details This function requests the GPS satellite status information	from the GPS subsystem. In config mode, this information is requested from the GPS subsystem immediately, which can cause a relatively long delay (250ms) before a reply is received. In measurement mode, the satellite status is regularly polled internally and the latest status is returned immediately when this function is called.
CMT_DLL_API XsensResultValue cmtGetGravityMagnitude(const int32_t instance, double* magnitude, const CmtDeviceId deviceId);		//!< \brief Retrieve the currently used magnitude of the gravity vector. \details The magnitude of the gravity vector is used to determine absolute acceleration from measured acceleration. \see cmtSetGravityMagnitude
CMT_DLL_API XsensResultValue cmtGetHardwareRevision (const int32_t instance, CmtVersion* revision, const CmtDeviceId deviceId);	//!< \brief Return Hardware revision. \details This function retrieves the hardware version of the device.
CMT_DLL_API XsensResultValue cmtGetHeading(const int32_t instance, double* heading, const CmtDeviceId deviceId);					//!< \brief Return the heading offset. \details This function retrieves the heading offset in radians used by the device. The valid range is -pi to +pi. The heading offset is used as a final correction on the output orientation. \see cmtSetHeading cmtGetMagneticDeclination cmtSetMagneticDeclination
CMT_DLL_API XsensResultValue cmtGetLastHwError(const int32_t instance, CmtDeviceId* did);											//!< \brief Return the last Hardware error code. \details This function returns the XsensResultValue of the last hardware problem (if any). \param did If any problems were found, the responsible device ID will be returned in this parameter.
CMT_DLL_API XsensResultValue cmtGetLatLonAlt(const int32_t instance, CmtVector* lla, const CmtDeviceId deviceId);				//!< \brief Set the current sensor position. \details Use this function to set the current position in latitude, longitude, altitude. \see cmtGetLatLonAlt
CMT_DLL_API XsensResultValue cmtGetLocationId (const int32_t instance, uint16_t* locationId, const CmtDeviceId deviceId);	//!< \brief Return the location ID of a sensor. \details This function retrieves the location ID stored in the device. \see cmtSetLocationId
CMT_DLL_API XsensResultValue cmtGetLogFileReadPos(const int32_t instance, XsensFilePos* pos, const CmtDeviceId deviceId);			//!< \brief Retrieve the read position of the log file containing the selected deviceId. \details This function will determine which log file contains the selected device ID and will return the current read position in that file in bytes from the start. \note The read and write positions of log files are completely independent of each other.
CMT_DLL_API XsensResultValue cmtGetLogFileSize(const int32_t instance, XsensFilePos* size, const CmtDeviceId deviceId);				//!< \brief Retrieve the size of the log file containing the selected deviceId. \details This function will determine which log file contains the selected device ID and then return the current size of that file in bytes.
CMT_DLL_API XsensResultValue cmtGetMagneticDeclination(const int32_t instance, double* declination, const CmtDeviceId deviceId);	//!< \brief Return the stored magnetic declination. \details This function retrieves the stored local magnetic declination in radians. The valid range is -pi to +pi. The magnetic declination is used in the sensor fusion process to determine the output orientation. \see cmtGetHeading cmtSetHeading cmtSetMagneticDeclination
CMT_DLL_API XsensResultValue cmtGetMainDeviceCount (const int32_t instance, uint16_t* count);								//!< \brief Retrieve the number of connected master devices. \details This function will retrieve the number of connected master devices (ports or files).
CMT_DLL_API XsensResultValue cmtGetMainDeviceId(const int32_t instance, CmtDeviceId* mainDeviceId, const CmtDeviceId deviceId);			//!< \brief Retrieve the Master Id given another Id. \details This function returns the main device Id, given any Device Id. When a single device is connected, this will return the supplied ID. When an Xbus Master is connected and one of the connected sensor's IDs is supplied, this will return the ID of the Xbus Master.
CMT_DLL_API XsensResultValue cmtGetMainDeviceIdFromIndex(const int32_t instance, CmtDeviceId* mainDeviceId, const uint16_t index);	//!< \brief Return the id of the device at index in the idList (skipping slave IDs) or 0 if not connected. \details This function differs from cmtGetXmDeviceId in that it will also return standalone connected MT devices \see cmtGetMainDeviceCount cmtGetMainDeviceId cmtGetMtCount cmtGetMtDeviceId cmtGetXmDeviceId
CMT_DLL_API XsensResultValue cmtGetMtCount(const int32_t instance, uint16_t* count);													//!< \brief Retrieve number of MT devices. \details This function will return the number of connected MT devices. Effectively, this returns the device count minus any Xbus Masters.
CMT_DLL_API XsensResultValue cmtGetMtDeviceId(const int32_t instance, CmtDeviceId* deviceId, const uint16_t index);			//!< \brief Retrieve the DeviceId of a device given its index in the MT list. \details This function retrieves the DeviceId for the device at the given index in the MT list. A valid index is in the range of 0 to the value supplied by cmtGetMtCount. When no items are connected, a 0 ID is supplied. \remarks When a device is connected or disconnected, the list of devices will be sorted again. This may change the index of any or all devices in the list. This function is similar to cmtGetDeviceId, but it ignores Xbus Masters. \see cmtGetDeviceId cmtGetXmDeviceId
CMT_DLL_API XsensResultValue cmtGetNextDataBundle (const int32_t instance);														//!< \brief Retrieve the next available data bundle. \details Inbound data from different devices is bundled into one object by CMT. This object is referred to as a bundle in CMT. These bundles are stored in an internal queue. The calling application can use this function to pop a bundle from the queue so it can access its data. See the cmtData... functions for a description on how to access the data in a bundle. If the Queue Mode is set to CMT_QM_LAST, the last-added item is made available by this function, otherwise the oldest available item is returned. If no new data bundles are available, the function will return an error, but the old data bundle will remain available. \see cmtSetQueueMode
CMT_DLL_API XsensResultValue cmtGetObjectAlignmentMatrix(const int32_t instance, CmtMatrix* matrix, const CmtDeviceId deviceId);				//!< \brief Retrieve the last stored object alignment matrix. \details Use this function to get the object matrix currently used as the sensor frame with respect to the reference frame. See the manual for more information on coordinate systems and object resets. \see cmtSetObjectAlignmentMatrix
#ifdef _WIN32
CMT_DLL_API XsensResultValue cmtGetPortNr(const int32_t instance, uint16_t* port, const CmtDeviceId deviceId);				//!< \brief Return the port number of a device. \details This function searches the connected ports for a device with the supplied device ID and then returns that port number. If CMT is reading from a log file, an error will be returned.
#endif
CMT_DLL_API XsensResultValue cmtGetPortName(const int32_t instance, char *portName, const CmtDeviceId deviceId);
CMT_DLL_API	XsensResultValue cmtGetProcessingFlags(const int32_t instance, uint16_t* processingFlags, const CmtDeviceId deviceId);	//!< \brief Return the current XKF processing options from a Motion Tracker. \details This function retrieves the current processing flags used by the XKF filter.
CMT_DLL_API XsensResultValue cmtGetProductCode (const int32_t instance, char* productCode, const CmtDeviceId deviceId);			//!< \brief Return product code. \details This function retrieves the product code of the given device. \param productCode The buffer that will store the product code. This buffer should be at least 20 bytes. \remarks The product code is NOT necessarily NULL-terminated. If it is less than 20 characters it will be, but if the product code is 20 characters it won't be. The recommended method is to create a buffer that is 21 bytes long and set the last byte to 0 before calling this function.
CMT_DLL_API XsensResultValue cmtGetQueueLength(const int32_t instance, uint32_t* length, int32_t queueId);						//!< \brief Return the length of a message queue. \details There are 2 queues in CMT that are relevant for applications to monitor. The most important is queue 0, which contains the fully processed data. This queue contains only complete data bundles. The other queue is queue 1, which contains incomplete data bundles and complete data bundles that have not yet been processed (if any processing is done on the host). \see cmtFlushDataBuffer
CMT_DLL_API XsensResultValue cmtGetQueueMode(const int32_t instance, CmtQueueMode* mode);											//!< \brief Return the queue mode. \details This function retrieves the current queue mode of the instance. The queue mode is either CMT_QM_FIFO or CMT_QM_LAST. \see cmtGetNextDataBundle cmtSetQueueMode
CMT_DLL_API const char* cmtGetResultText(const XsensResultValue result);														//!< \brief Translate result code into text. \details Use this function to translate an XsensResultValue code into a readable (english) text. The function returns a pointer to statically allocated text, so do NOT change the data at the returned pointer.
CMT_DLL_API XsensResultValue cmtGetSampleFrequency(const int32_t instance, uint16_t* freq, const CmtDeviceId deviceId);									//!< \brief Return current sample frequency. \details This function determines the sample frequency of the device from the internal sample rate and skip factor and returns it in freq. For devices connected to an Xbus Master, the values used by the Xbus Master are returned. \see cmtSetDeviceMode
CMT_DLL_API XsensResultValue cmtGetScenario(const int32_t instance, uint8_t* scenarioType, uint8_t* scenarioVersion, const CmtDeviceId deviceId);			//!< \brief Get the currently active scenario from a Motion Tracker. \details This function retrieves the scenario currently used by the specified device when it is doing its own processing. When using software calibration and filtering, use cmtGetSoftwareScenario instead \see cmtSetScenario cmtGetAvailableScenarios cmtGetAvailableSoftwareScenarios cmtGetSoftwareScenario cmtSetSoftwareScenario cmtUploadScenario
CMT_DLL_API XsensResultValue cmtGetSoftwareScenario(const int32_t instance, uint8_t* scenarioType, uint8_t* scenarioVersion, const CmtDeviceId deviceId);	//!< \brief Get the currently active software scenario. \details	This function retrieves the scenario currently used for the specified device when using software calibration and filtering. When not using software calibration and filtering, use cmtGetScenario instead. \see cmtGetScenario cmtSetScenario cmtGetAvailableScenarios cmtGetAvailableSoftwareScenarios cmtSetSoftwareScenario cmtUploadScenario cmtLoadScenarios
CMT_DLL_API XsensResultValue cmtGetSerialBaudrate (const int32_t instance, uint32_t* baudrate, const CmtDeviceId deviceId);										//!< \brief Return the baud rate used for serial communication. \details This function determines to which port the selected device ID is connected and then retrieves the baud rate that is used when the device is connected by a serial connection. In most cases this is the same as cmtGetBaudrate, but when an Xbus Master is connected by a Bluetooth connection, it doesn't have to be. \see cmtSetSerialBaudrate cmtSetBaudrate cmtGetBaudrate
CMT_DLL_API XsensResultValue cmtGetSuitableScenarios (const int32_t instance, CmtScenario *scenarios, CmtDeviceId deviceId);			//!< \brief Return a list of scenarios that are suitable for uploading
CMT_DLL_API XsensResultValue cmtGetSyncInSettings (const int32_t instance, CmtSyncInSettings* settings, const CmtDeviceId deviceId);	//!< \brief Retrieve the inbound synchronization settings of an MT device. \details This function retrieves the current inbound synchronization settings of the MT device (sync mode, skip factor and offset). This function does not work for Xbus Masters and should not be used for sensors connected to an Xbus Master. \see cmtSetSyncInSettings cmtGetSyncOutSettings cmtSetSyncOutSettings cmtGetSyncMode cmtSetSyncMode
CMT_DLL_API XsensResultValue cmtGetSyncInMode(const int32_t instance, uint16_t* mode, const CmtDeviceId deviceId);													//!< \brief Retrieve the inbound synchronization mode of an MT device. \details This function retrieves the current inbound synchronization mode of the MT device. This function does not work for Xbus Masters and should not be used for sensors connected to an Xbus Master. \see cmtSetSyncInMode cmtSetSyncInSettings cmtGetSyncInSettings cmtGetSyncInSkipFactor cmtSetSyncInSkipFactor cmtGetSyncInOffset cmtSetSyncInOffset cmtGetSyncOutSettings cmtSetSyncOutSettings cmtGetSyncMode cmtSetSyncMode
CMT_DLL_API XsensResultValue cmtGetSyncInSkipFactor(const int32_t instance, uint16_t* skipFactor, const CmtDeviceId deviceId);										//!< \brief Retrieve the inbound synchronization skip factor of an MT device. \details This function retrieves the current inbound synchronization skip factor of the MT device. This function does not work for Xbus Masters and should not be used for sensors connected to an Xbus Master. \see cmtSetSyncInSkipFactor cmtSetSyncInSettings cmtGetSyncInSettings cmtGetSyncInOffset cmtSetSyncInOffset cmtGetSyncOutSettings cmtSetSyncOutSettings cmtGetSyncMode cmtSetSyncMode
CMT_DLL_API XsensResultValue cmtGetSyncInOffset(const int32_t instance, uint32_t* offset, const CmtDeviceId deviceId);												//!< \brief Retrieve the inbound synchronization offset of an MT device. \details This function retrieves the current inbound synchronization offset of the MT device. This function does not work for Xbus Masters and should not be used for sensors connected to an Xbus Master. \see cmtSetSyncInOffset cmtSetSyncInSettings cmtGetSyncInSettings cmtGetSyncInSkipFactor cmtSetSyncInSkipFactor cmtGetSyncOutSettings cmtSetSyncOutSettings cmtGetSyncMode cmtSetSyncMode
CMT_DLL_API XsensResultValue cmtGetSyncMode(const int32_t instance, CmtXmSyncMode* mode, const CmtDeviceId deviceId);					//!< \brief Retrieve the sync mode of the Xbus Master. \details This function requests the current synchonization mode used by the specified Xbus Master. This function is not valid for MT devices. \see cmtSetSyncMode cmtGetSyncInSettings cmtSetSyncInSettings cmtGetSyncOutSettings cmtSetSyncOutSettings
CMT_DLL_API XsensResultValue cmtGetSyncOutSettings (const int32_t instance, CmtSyncOutSettings* settings, const CmtDeviceId deviceId);	//!< \brief Retrieve the outbound synchronization settings of an MT device. \details This function retrieves the current outbound synchronization settings of the MT device (sync mode, skip factor, offset and pulse width). This function does not work for Xbus Masters and should not be used for sensors connected to an Xbus Master. \see cmtSetSyncOutSettings cmtGetSyncInSettings cmtSetSyncInSettings cmtGetSyncMode cmtSetSyncMode
CMT_DLL_API XsensResultValue cmtGetSyncOutMode(const int32_t instance, uint16_t* mode, const CmtDeviceId deviceId);																		//!< \brief Retrieve the outbound synchronization mode of an MT device. \details This function retrieves the current outbound synchronization mode of the MT device. This function does not work for Xbus Masters and should not be used for sensors connected to an Xbus Master. \see cmtSetSyncOutMode cmtSetSyncOutSettings cmtGetSyncOutSettings cmtGetSyncOutPulseWidth cmtSetSyncOutPulseWidth cmtGetSyncOutSkipFactor cmtSetSyncOutSkipFactor cmtGetSyncOutOffset cmtSetSyncOutOffset cmtGetSyncInSettings cmtSetSyncInSettings cmtGetSyncMode cmtSetSyncMode
CMT_DLL_API XsensResultValue cmtGetSyncOutPulseWidth(const int32_t instance, uint32_t* pulseWidth, const CmtDeviceId deviceId);																//!< \brief Retrieve the outbound synchronization pulse width of an MT device. \details This function retrieves the current outbound synchronization pulse width of the MT device. This function does not work for Xbus Masters and should not be used for sensors connected to an Xbus Master. \see cmtSetSyncOutPulseWidth cmtSetSyncOutSettings cmtGetSyncOutSettings cmtSetSyncOutMode cmtGetSyncOutMode cmtGetSyncOutSkipFactor cmtSetSyncOutSkipFactor cmtGetSyncOutOffset cmtSetSyncOutOffset cmtGetSyncInSettings cmtSetSyncInSettings cmtGetSyncMode cmtSetSyncMode
CMT_DLL_API XsensResultValue cmtGetSyncOutSkipFactor(const int32_t instance, uint16_t* skipFactor, const CmtDeviceId deviceId);															//!< \brief Retrieve the outbound synchronization skip factor of an MT device. \details This function retrieves the current outbound synchronization skip factor of the MT device. This function does not work for Xbus Masters and should not be used for sensors connected to an Xbus Master. \see cmtStSyncOutSkipFactor cmtSetSyncOutSettings cmtGetSyncOutSettings cmtSetSyncOutMode cmtGetSyncOutMode cmtGetSyncOutPulseWidth cmtSetSyncOutPulseWidth cmtGetSyncOutOffset cmtSetSyncOutOffset cmtGetSyncInSettings cmtSetSyncInSettings cmtGetSyncMode cmtSetSyncMode
CMT_DLL_API XsensResultValue cmtGetSyncOutOffset(const int32_t instance, uint32_t* offset, const CmtDeviceId deviceId);																		//!< \brief Retrieve the outbound synchronization offset of an MT device. \details This function retrieves the current outbound synchronization offset of the MT device. This function does not work for Xbus Masters and should not be used for sensors connected to an Xbus Master. \see cmtSetSyncOutOffset cmtSetSyncOutSettings cmtGetSyncOutSettings cmtSetSyncOutMode cmtGetSyncOutMode cmtGetSyncOutPulseWidth cmtSetSyncOutPulseWidth cmtGetSyncOutSkipFactor cmtSetSyncOutSkipFactor cmtGetSyncInSettings cmtSetSyncInSettings cmtGetSyncMode cmtSetSyncMode
CMT_DLL_API XsensResultValue cmtGetTimeoutConfig (const int32_t instance, uint32_t* timeout);										//!< \brief Return the configuration mode timeout. \details When in config mode, the system uses a different message timeout than in measurement mode, since configuration messages can take longer to process than measurement mode messages. This function can be used to determine the current config mode timeout in ms. \see cmtSetTimeoutConfig cmtGetTimeoutMeasurement cmtSetTimeoutMeasurement cmtSetTimeoutData
CMT_DLL_API XsensResultValue cmtGetTimeoutMeasurement (const int32_t instance, uint32_t* timeout);						//!< \brief Return the measurement mode timeout. \details When in measurement mode, the system uses a different message timeout than in config mode, since measurement mode messages should be faster to process than config mode messages. This function can be used to determine the current measurement mode timeout in ms. \see cmtSetTimeoutMeasurement cmtGetTimeoutConfig cmtSetTimeoutConfig cmtSetTimeoutData
CMT_DLL_API XsensResultValue cmtGetTransmissionDelay(const int32_t instance, uint16_t* delay, const CmtDeviceId deviceId);			//!< \brief Return the RS485 acknowledge transmission delay of the device. \details This function returns the RS485 acknowledge transmission delay of the device. The RS485 acknowledge transmission delay determines the minimal delay to response on request messages. See the low-level communication documentation for more details. \see cmtSetAckTransmissionDelay
CMT_DLL_API XsensResultValue cmtGetUtcTime(const int32_t instance, CmtUtcTime* utc, const CmtDeviceId deviceId);				//!< \brief Return the UTC time of the last received sample. \details This function is only valid for MTi-G sensors. In measurement mode it will retrieve the UTC time of the last received sample. In Config mode, it will retrieve the most recent UTC time. In config mode, the time returned will be requested diectly from the GPS subsystem, with several layers of messaging between the original source and the host. In measurement mode, the UTC time is requested from the GPS subsystem periodically and estimated (with a very high precision) for the remaining samples. So in config mode, the time will probably jitter more than in measurement mode.
CMT_DLL_API XsensResultValue cmtGetXmDeviceId(const int32_t instance, CmtDeviceId* deviceId, const uint16_t index);		//!< \brief Retrieve the DeviceId of an Xbus Master device given its index. \details This function retrieves the DeviceId for the device at the given index in the Xbus Master list. Only Xbus Masters are indexed for this function. When no devices are connected, a 0 ID is supplied. \remarks When a device is connected or disconnected, the list of devices will be sorted again. This may change the index of any or all devices in the list. This function is similar to cmtGetDeviceId, but it ignores all non-Xbus Masters. \see cmtGetDeviceId cmtGetMtDeviceId
CMT_DLL_API XsensResultValue cmtGetXmOutputMode(const int32_t instance, uint8_t* mode, const CmtDeviceId deviceId);		//!< \brief Return the dual-output mode of the XM. \details This function retrieves the dual mode output mode of the Xbus Master. The dual output mode of the Xbus Master defines whether it will send data on the serial connection (at the same baud rate as the bluetooth connection) when it is connected via Bluetooth. When set to 0, the serial communication is disabled, data is sent over the serial bus. \note When dual-mode is enabled, the Xbus Master will NOT receive command messages on the serial bus. \see cmtSetXmOutputMode

CMT_DLL_API XsensResultValue cmtGotoConfig (const int32_t instance);							//!< \brief Place all connected devices into Configuration Mode. \details This function places the sensors in configuration mode in the appropriate order as defined by their synchronization settings. \note When destroying a CMT instance, it will automatically call this function internally before shutting down. \note This function has no effect when reading from a log file
CMT_DLL_API XsensResultValue cmtGotoMeasurement (const int32_t instance);						//!< \brief Place all connected devices into Measurement Mode. \details This function places the sensors in measurement mode in the appropriate order as defined by their synchronization settings. \note This function has no effect when reading from a log file
CMT_DLL_API int32_t cmtIsMeasuring(const int32_t instance);										//!< \brief Return whether the CMT instance is currently measuring (non-zero) or not (zero).
CMT_DLL_API int32_t cmtIsMessageQueueEmpty(const int32_t instance);								//!< \brief Return whether the inbound data queue (queue 0) is empty (non-zero) or not (zero). \note This function is faster than checking the actual length of the queue with cmtGetQueueLength. It is also directly usable in boolean equations.

CMT_DLL_API bool cmtIdIsMtig(const CmtDeviceId did);		//!< \brief Return whether the supplied device ID belongs to an MTi-G. \note This function does not require you to supply an instance id
CMT_DLL_API bool cmtIdIsMtix(const CmtDeviceId did);		//!< \brief Return whether the supplied device ID belongs to an MTi or MTx device. \note This function does not require you to supply an instance id
CMT_DLL_API bool cmtIdIsMtxXbus(const CmtDeviceId did);		//!< \brief Return whether the supplied device ID belongs to an MTx Xbus device. \note This function does not require you to supply an instance id
CMT_DLL_API bool cmtIdIsXm(const CmtDeviceId did);			//!< \brief Return whether the supplied device ID belongs to an Xbus Master. \note This function does not require you to supply an instance id

CMT_DLL_API XsensResultValue cmtLoadScenarios(const int32_t instance, const char* filename);	//!< \brief Load scenario definitions from a file. \details Use this function to read the scenario definitions from the given (xsb) file. When no file is loaded, only the default scenario is available for software filtering. \remarks Supply NULL or an empty "" filename to unload scenario definitions. \see cmtGetSoftwareScenario cmtGetAvailableSoftwareScenarios cmtSetSoftwareScenario cmtUploadScenario
CMT_DLL_API XsensResultValue cmtLoadScenariosW(const int32_t instance, const wchar_t* filename);	//!< \brief Load scenario definitions from a file. \details Use this function to read the scenario definitions from the given (xsb) file. When no file is loaded, only the default scenario is available for software filtering. \remarks Supply NULL or an empty "" filename to unload scenario definitions. \see cmtGetSoftwareScenario cmtGetAvailableSoftwareScenarios cmtSetSoftwareScenario cmtUploadScenario
CMT_DLL_API XsensResultValue cmtOpenLogFile(const int32_t instance, const char * filename);																//!< \brief Open a log file for input. \details This function opens the indicated log file for reading. The function will fail if a serial connection is currently open in the CMT. \param filename The name of the file to open. It is recommended to use a fully qualified path+filename. \see cmtGetLogFileReadPos cmtGetLogFileSize cmtOpenPort cmtCloseLogFile cmtCreateLogFile
CMT_DLL_API XsensResultValue cmtOpenLogFileW(const int32_t instance, const wchar_t* filename);																//!< \brief Open a log file for input. \details This function opens the indicated log file for reading. The function will fail if a serial connection is currently open in the CMT. \param filename The name of the file to open. It is recommended to use a fully qualified path+filename. \see cmtGetLogFileReadPos cmtGetLogFileSize cmtOpenPort cmtCloseLogFile cmtCreateLogFile
#ifdef _WIN32
CMT_DLL_API XsensResultValue cmtOpenPort(const int32_t instance, const uint16_t portNumber, const uint32_t baudRate = CMT_DEFAULT_BAUD_RATE);	//!< \brief Open a communication channel to the given COM port number. \details This function opens the indicated COM port for communication. If no Xsens device is found on the port, the function will fail. \see cmtScanPorts cmtScanPort cmtClosePort
CMT_DLL_API XsensResultValue cmtOpenPortNoWakeup(const int32_t instance, const uint16_t portNumber, const uint32_t baudRate);
#else
CMT_DLL_API XsensResultValue cmtOpenPort(const int32_t instance, const char *portName, const uint32_t baudRate = CMT_DEFAULT_BAUD_RATE);
CMT_DLL_API XsensResultValue cmtOpenPortNoWakeup(const int32_t instance, const char *portName, const uint32_t baudRate = CMT_DEFAULT_BAUD_RATE);
#endif
CMT_DLL_API XsensResultValue cmtPeekMessageId(const int32_t instance, const CmtDeviceId deviceId, uint8_t* messageId);								//!< \brief Peek(take a look) at the message ID of the next message. \details This function can only be used when reading from a log file. It will find the next message in the file and place its message ID in the messageId parameter. Afterwards, the read position of the file will be restored. \remarks This function is mostly useful when dealing with a file that has more than just data messages. By using the peek function, it is possible to decide whether a cmtGetNextDataBundle should be called or for example cmtGetBatteryLevel. When the peek function is not used and for example cmtGetBatteryLevel is called, all messages between the current read position and the first battery level message in the file will be skipped. An alternative would be to get the read position, call the desired function (get battery level) and restore the read position, but then the moment at which the (battery level, UTC time, satellite info, etc) data becomes available is not known.
CMT_DLL_API XsensResultValue cmtReadRawMessage(const int32_t instance, const CmtDeviceId deviceId, uint8_t* messageBuffer);							//!< \brief Read a raw message. \details This function can only be used when reading from a log file. It will find the next message in the file and place its message ID in the messageId parameter. Afterwards, the read position of the file will be restored. \remarks This function is mostly useful when dealing with a file that has more than just data messages. By using the peek function, it is possible to decide whether a cmtGetNextDataBundle should be called or for example cmtGetBatteryLevel. When the peek function is not used and for example cmtGetBatteryLevel is called, all messages between the current read position and the first battery level message in the file will be skipped. An alternative would be to get the read position, call the desired function (get battery level) and restore the read position, but then the moment at which the (battery level, UTC time, satellite info, etc) data becomes available is not known.
CMT_DLL_API XsensResultValue cmtReset(const int32_t instance);																		//!< \brief Reset all connected sensors. \details This function calls the reset function for all connected devices.
CMT_DLL_API XsensResultValue cmtResetDroppedPacketCounts(const int32_t instance);													//!< \brief Reset the dropped packet counters. \details This function resets the dropped packet counters that are kept for each connected port. \see cmtGetDroppedPacketCount
CMT_DLL_API XsensResultValue cmtResetLogFileReadPos(const int32_t instance);														//!< \brief Reset the read position of all open log files. \details	This function restarts reading all open log files from the start of the file.
CMT_DLL_API XsensResultValue cmtResetOrientation(const int32_t instance, const CmtResetMethod method, const CmtDeviceId deviceId);	//!< \brief Perform an orientation reset on a device. \details This function performs an orientation reset. See the MT documentation for more information about Orientation resets. \note If you wish to save the setting to the device, perform the CMT_RESETORIENTATION_STORE operation while in Config mode.
CMT_DLL_API XsensResultValue cmtRunSelfTest(const int32_t instance, uint16_t* result, const CmtDeviceId deviceId);									//!< \brief Let the MT perform a self test. The status of the selftest can be viewed in the status byte. The function will always return immediately.
CMT_DLL_API XsensResultValue cmtSetNoRotation(const int32_t instance, const uint16_t duration, const CmtDeviceId deviceId);	//!< \brief Initiates the XKF3 'no rotation' update procedure. \details This function Initiates the XKF3 'no rotation' update procedure. See the MT documentation for more information. \note Note that this message does not convey any status. Bits 3-4 of the MT status byte are used instead. Only valid in measurement mode
CMT_DLL_API XsensResultValue cmtRestoreFactoryDefaults(const int32_t instance, const CmtDeviceId deviceId);														//!< \brief Restore the device to factory default settings. \details This function completely restores the selected device to the default settings (115k2 baud rate, 100Hz sample frequency).
#ifdef _WIN32
CMT_DLL_API XsensResultValue cmtScanPort(uint16_t portNr, CmtPortInfo* portInfo, uint32_t baudrate = 0, uint32_t singleScanTimeout = 1000, uint32_t scanTries = 1);		//!< \brief Scan the given port for connected Xsens devices. \details This function scans a single ports for connected Xsens devices. The function does not require an instance and functions for non-open ports only. \param baudrate Can be set to 0 to scan at all supported baud rates or limited to a single baud rate. \param portInfo A pointer to the CmtPortInfo object that will store the port information. \param singleScanTimeout This parameter determines the timeout value to use when scanning at a single baud rate for Xsens devices. Reducing the timeout will make the scan go faster, but the chance of failing to detect a device increases. \param scanTries The number of attempts to put a device in config mode. For RS485 sensors, increasing this value will increase the chance of detection, but it will also increase the scan-time.
#else
CMT_DLL_API XsensResultValue cmtScanPort(const char *portName, CmtPortInfo *portInfo, uint32_t baudrate = 0, uint32_t singleScanTimeout = 1000, uint32_t scanTries = 1);
#endif
CMT_DLL_API XsensResultValue cmtScanPorts(CmtPortInfo* portInfo, uint32_t* count, uint32_t baudrate = 0, uint32_t singleScanTimeout = 1000, uint32_t scanTries = 1);		//!< \brief Scan for connectable ports. \details This function scans all ports for connected Xsens devices. The function does not require an instance and functions for non-open ports only. \param baudrate Can be set to 0 to scan at all supported baud rates or limited to a single baud rate. \param portInfo This buffer should be able to hold at least 256 CmtPortInfo objects. \param singleScanTimeout This parameter determines the timeout value to use when scanning a single port at a single baud rate for Xsens devices. Reducing the timeout will make the scan go faster, but the chance of failing to detect a device increases. \param scanTries The number of attempts to put a device in config mode. For RS485 sensors, increasing this value will increase the chance of detection, but it will also increase the scan-time.

/*! \brief Send a custom message and optionally wait for a reply.
	\details Use this function to send a customized message to the given device. All public messages that the Xsens devices support are included as a function in CMT, so sending a custom message should rarely be necessary. Using this interface it is possible to send custom messages if you have a customized firmware or if you have customized hardware connected to the Xbus.

	\param deviceId The ID of the device to send the message to. Internally, CMT figures out the correct port and bus Id from this value.
	\param messageId The message ID of the message. See the low level communication documentation for typical message IDs.
	\param dataLength The size of the supplied dataBuffer. For most messages that request something, this is 0.
	\param dataBuffer The buffer that contains the data to be sent in the correct byte-order (big-endian). If dataLength is 0, this field may be NULL.
	\param waitForThisMsgId When non-zero, CMT will wait until a message with this message Id or an error is received. When in measurement mode, normal data is still received and queued as normal.
	\param resultBuffer When waitForThisMsgId is non-zero and this pointer is non-NULL, the data in the reply from the device is stored in this buffer. Make sure that the buffer is large enough to hold the data.
*/
CMT_DLL_API XsensResultValue cmtSendCustomMessage(const int32_t instance, const CmtDeviceId deviceId, const uint8_t messageId, const uint16_t dataLength, const uint8_t* dataBuffer, const uint8_t waitForThisMsgId, uint8_t* resultBuffer);

CMT_DLL_API XsensResultValue cmtSetBaudrate(const int32_t instance, const uint32_t baudrate, const CmtDeviceId deviceId);		//!< \brief Set the baud rate for the given device. \details This function sets the baud rate for the selected device and reconnects to it at the new baud rate if successful. This function can only be used on the main device of a port.
CMT_DLL_API XsensResultValue cmtSetBluetoothState(const int32_t instance, const int32_t enabled, const CmtDeviceId deviceId);			//!< \brief Set the Bluetooth state of an Xbus Master. \details This function sets the state of the bluetooth communication to on (non-zero) or off (zero).
CMT_DLL_API XsensResultValue cmtSetBusPowerState(const int32_t instance, const int32_t enabled, const CmtDeviceId deviceId);			//!< \brief Switch the XM bus power on or off. \details Use this function to switch the Xbus Master Xbus power on or off. \remarks This function can be used to save a lot of power when not measuring, while still keeping a connection to the system. However, there is a relatively long startup time when restoring power as the sensors are reinitialized.

CMT_DLL_API XsensResultValue cmtRegisterCallback(const int32_t instance, CmtCallbackFunction func, CmtCallbackType type, void* userParam);	//!< \brief Register a user function as a callback function for the specified type. \details This function should return as soon as possible to prevent possible data log in Cmt.
CMT_DLL_API XsensResultValue cmtUnregisterCallback(const int32_t instance, CmtCallbackFunction func, CmtCallbackType type);					//!< \brief Unregister a user function as a callback function for the specified type.
/*! \brief Set the complete output mode of a device. 
\details This function updates the complete output mode of the specified device. It only updates values that are different than those reported by the device. 
When setting the sample frequency of a device connected to an Xbus Master, the sample frequency of the Xbus Master is updated instead.

\param mode Set the mode of the device. Output mode values can be obtained through the OUTPUTMODE_* functions.
\param settings Set mode specific settings for the device. The settings values can be obtained through the OUTPUTSETTINGS_* functions.
\param frequency Set the output frequency of the device in Hz. 
\param deviceId The ID of the device that has to be updated.

\see cmtGetDeviceMode cmtGetDeviceMode2 cmtSetDeviceMode2
*/
CMT_DLL_API XsensResultValue cmtSetDeviceMode(const int32_t instance, const CmtOutputMode mode, const CmtOutputSettings settings, const uint16_t frequency, const CmtDeviceId deviceId);
/*! \brief Set the complete output mode2 of a device. 
\details This function updates the complete output mode of the specified device. It only updates values that are different than those reported by the device. 
When setting the sample frequency of a device connected to an Xbus Master, the sample frequency of the Xbus Master is updated instead.

\param mode Set the mode of the device. Output mode values can be obtained through the OUTPUTMODE_* functions.
\param settings Set mode specific settings for the device. The settings values can be obtained through the OUTPUTSETTINGS_* functions.
\param period Set the sample period for the device. The period values can be obtained through the PERIOD_* functions.
\param skipFactor Set the output skip factor for the device. A sample frequency of 100 Hz with a skip factor of 9 will result in 10 Hz output.
\param deviceId The ID of the device that has to be updated.

\see cmtGetDeviceMode2 cmtGetDeviceMode cmtSetDeviceMode
*/
CMT_DLL_API XsensResultValue cmtSetDeviceMode2(const int32_t instance, const CmtOutputMode mode, const CmtOutputSettings settings, const uint16_t period, const uint16_t skipFactor, const CmtDeviceId deviceId);
CMT_DLL_API XsensResultValue cmtSetErrorMode(const int32_t instance, const uint16_t mode, const CmtDeviceId deviceId);					//!< \brief Set the error mode of the device. \details This function sets the error mode of the device. The error mode determines how the device handles errors. See the low-level communication documentation for more details. \see cmtGetErrorMode
CMT_DLL_API XsensResultValue cmtSetGpsLeverArm(const int32_t instance, const CmtVector arm, const CmtDeviceId deviceId);						//!< \brief Set the currently used GPS lever arm. \details Use this function to set the vector currently used as the GPS lever arm. The GPS lever arm is the relative position of the GPS antenna to the MTi-G unit. The arm is specified in the object coordinate system and in meters. See the manual for more information on coordinate systems, alignment resets and the lever arm. \see cmtGetGpsLeverArm
CMT_DLL_API XsensResultValue cmtSetGravityMagnitude(const int32_t instance, const double magnitude, const CmtDeviceId deviceId);				//!< \brief Set the currently used magnitude of the gravity vector. \details The magnitude of the gravity vector is used to determine absolute acceleration from measured acceleration. Use this function to set tit to a custom value in m/s2. The default value of 9.812687357684514m/s2 should be fine for most places on earth, but in some cases the gravity may be drastically different from standard Earth gravity (eg. space, deep subterranean/submarine, polar regions, gravitational anomalies. See also <A HREF="http://www.abc.net.au/science/news/stories/s911917.htm">http://www.abc.net.au/science/news/stories/s911917.htm</A> and <A HREF="http://en.wikipedia.org/wiki/Earth%27s_gravity#Comparative_gravities_in_various_cities_around_the_world">http://en.wikipedia.org/wiki/Earth%27s_gravity#Comparative_gravities_in_various_cities_around_the_world</A>) \note Correct orientation output cannot be guaranteed when an incorrect value is supplied for the gravity. Changing this value is at your own risk. \see cmtGetGravityMagnitude \remarks In future versions, the MTi-G may be able to estimate the local gravity magnitude on its own.
CMT_DLL_API XsensResultValue cmtSetHeading(const int32_t instance, const double heading, const CmtDeviceId deviceId);							//!< \brief Set the heading offset. \details This function sets the heading offset in radians used by the device. The valid range is -pi to +pi. The heading offset is used as a final correction on the output orientation. \see cmtGetHeading cmtGetMagneticDeclination cmtSetMagneticDeclination
CMT_DLL_API XsensResultValue cmtSetInterpolationMode(const int32_t instance, const int32_t mode);													//!< \brief Set the interpolation mode for missed data. \details In some situations it's possible to miss data due to an unreliable communication system. This function can be used to tell CMT how to handle missing data. The options are 2: missing data is only reported through missing samples callback function. Then ignored, 1: missing data is reported through missing samples callback function and then linearly interpolated between the samples on either side of the missing data sample or 0: missing data remains as gaps in the data stream. \note When connected to multiple ports and using mode 0, losing a sample from one port will mean that the corresponding samples from the other ports are also dropped.
CMT_DLL_API XsensResultValue cmtSetLatLonAlt(const int32_t instance, const CmtVector lla, const CmtDeviceId deviceId);						//!< \brief Retrieve the last stored sensor position. \details This function retrieves the last stored position in latitude, longitude, altitude. \see cmtSetLatLonAlt
CMT_DLL_API XsensResultValue cmtSetLocationId (const int32_t instance, const uint16_t locationId, const CmtDeviceId deviceId);			//!< \brief Set the location ID of a sensor. \details This function sets the location ID stored in the device. \see cmtGetLocationId
CMT_DLL_API XsensResultValue cmtSetLogMode(const int32_t instance, const int32_t enabled, const CmtDeviceId deviceId);							//!< \brief Set the logging mode. \details This function sets the logging mode to enabled (non-zero) or disabled (zero). Only the logging mode of the port that hosts the given device Id is affected. When enabled, all received messages are logged to the file.
CMT_DLL_API XsensResultValue cmtSetMagneticDeclination(const int32_t instance, double declination, const CmtDeviceId deviceId);				//!< \brief Set the stored magnetic declination. \details This function sets the stored local magnetic declination in radians. The valid range is -pi to +pi. The magnetic declination is used in the sensor fusion process to determine the output orientation. \see cmtGetHeading cmtSetHeading cmtGetMagneticDeclination
CMT_DLL_API XsensResultValue cmtSetObjectAlignmentMatrix(const int32_t instance, const CmtMatrix matrix, const CmtDeviceId deviceId);						//!< \brief Set the currently used object alignment matrix. \details Use this function to set the object matrix to be used as the sensor frame with respect to the reference frame. See the manual for more information on coordinate systems and object resets. \see cmtGetObjectAlignmentMatrix
CMT_DLL_API XsensResultValue cmtSetProcessingFlags(const int32_t instance, const uint16_t processingFlags, const CmtDeviceId deviceId);	//!< \brief Set the XKF processing options of a Motion Tracker. \details This function sets the processing flags used by the XKF filter.
CMT_DLL_API XsensResultValue cmtSetQueueMode(const int32_t instance, CmtQueueMode mode);														//!< \brief Set the queue mode. \details This function sets the current queue mode of the instance. The queue mode is either CMT_QM_FIFO or CMT_QM_LAST. \see cmtGetNextDataBundle cmtGetQueueMode
CMT_DLL_API XsensResultValue cmtSetScenario(const int32_t instance, const uint8_t scenarioType, const CmtDeviceId deviceId);				//!< \brief Specify the scenario to use in the sensor. \details This function specifies the scenario that the sensor should use for processing data. This must be one of the scenarios in the list supplied by cmtGetAvailableScenarios. \see cmtGetScenario cmtGetAvailableScenarios cmtGetAvailableSoftwareScenarios cmtGetSoftwareScenario cmtSetSoftwareScenario
CMT_DLL_API XsensResultValue cmtSetSoftwareCalibration(const int32_t instance, const int32_t enabled);											//!< \brief Switch software-based calibration on or off. \details This function switches software-based calibration on(non-zero) or off(zero). When software-based calibration is enabled, raw data is automatically converted into calibrated data by CMT. This requires a valid log file or a valid CMT device on a port.
CMT_DLL_API XsensResultValue cmtSetSoftwareScenario(const int32_t instance, const uint8_t scenarioType, const CmtDeviceId deviceId);		//!< \brief Specify the scenario to use for software processing. \details This function specifies the scenario to use for processing data when using software calibration and filtering. This must be one of the scenarios in the list supplied by cmtGetAvailableSoftwareScenarios. \see cmtGetScenario cmtGetAvailableScenarios cmtGetAvailableSoftwareScenarios cmtGetSoftwareScenario cmtSetSoftwareScenario cmtLoadScenarios
CMT_DLL_API XsensResultValue cmtSetSoftwareXkf3Filtering(const int32_t instance, const int32_t enabled);											//!< \brief Switch software-based XKF-3 filtering on or off. \details This function switches software-based XKF-3 filtering on(non-zero) or off(zero). When XKF-3 filtering is performed on the data, orientation data is automatically computed from raw data. This function requires software calibration to be on as well. The scenario to be used can be set with cmtSetSoftwareScenario \note XKF-3 filtering is only available for data sources that supply RAW (uncalibrated) data. \see cmtGetAvailableSoftwareScenarios cmtGetSoftwareScenario cmtSetSoftwareScenario
CMT_DLL_API XsensResultValue cmtSetSoftwareXkf6Filtering(const int32_t instance, const int32_t enabled);											//!< \brief Switch software-based XKF-6 filtering on or off. \details This function switches software-based XKF-6 filtering on(non-zero) or off(zero). When XKF-6 filtering is performed on the data, orientation, position and velocity data are automatically computed from raw data. This function requires software calibration to be on as well. The scenario to be used can be set with cmtSetSoftwareScenario \note XKF-6 filtering is only available for data sources that supply RAW (uncalibrated) data and GPS PVT data, such as an MTi-G. \see cmtGetAvailableSoftwareScenarios cmtGetSoftwareScenario cmtSetSoftwareScenario
CMT_DLL_API XsensResultValue cmtSetSyncInSettings(const int32_t instance, const CmtSyncInSettings settings, const CmtDeviceId deviceId);		//!< \brief Set the inbound synchronization settings of an MT device. \details This function sets the current inbound synchronization settings of the MT device (sync mode, skip factor and offset). This function does not work for Xbus Masters and should not be used for sensors connected to an Xbus Master. \see cmtGetSyncInSettings cmtGetSyncInMode cmtSetSyncInMode cmtGetSyncInSkipFactor cmtSetSyncInSkipFactor cmtGetSyncInOffset cmtSetSyncInOffset cmtGetSyncOutSettings cmtSetSyncOutSettings cmtGetSyncMode cmtSetSyncMode
CMT_DLL_API XsensResultValue cmtSetSyncInMode (const int32_t instance, const uint16_t mode, const CmtDeviceId deviceId);		//!< \brief Set the inbound synchronization mode of an MT device. \details This function sets the current inbound synchronization mode of the MT device. This function does not work for Xbus Masters and should not be used for sensors connected to an Xbus Master. \see cmtGetSyncInMode cmtSetSyncInSettings cmtGetSyncInSettings cmtGetSyncInSkipFactor cmtSetSyncInSkipFactor cmtGetSyncInOffset cmtSetSyncInOffset cmtGetSyncOutSettings cmtSetSyncOutSettings cmtGetSyncMode cmtSetSyncMode
CMT_DLL_API XsensResultValue cmtSetSyncInSkipFactor (const int32_t instance, const uint16_t skipFactor, const CmtDeviceId deviceId);		//!< \brief Set the inbound synchronization skip factor of an MT device. \details This function sets the current inbound synchronization skip factor of the MT device. This function does not work for Xbus Masters and should not be used for sensors connected to an Xbus Master. \see cmtGetSyncInSkipFactor cmtSetSyncInSettings cmtGetSyncInSettings cmtGetSyncInOffset cmtSetSyncInOffset cmtGetSyncOutSettings cmtSetSyncOutSettings cmtGetSyncMode cmtSetSyncMode
CMT_DLL_API XsensResultValue cmtSetSyncInOffset (const int32_t instance, const uint32_t offset, const CmtDeviceId deviceId);			//!< \brief Set the inbound synchronization offset of an MT device. \details This function sets the current inbound synchronization offset of the MT device. This function does not work for Xbus Masters and should not be used for sensors connected to an Xbus Master. \see cmtGetSyncInOffset cmtSetSyncInSettings cmtGetSyncInSettings cmtGetSyncInSkipFactor cmtSetSyncInSkipFactor cmtGetSyncOutSettings cmtSetSyncOutSettings cmtGetSyncMode cmtSetSyncMode
CMT_DLL_API XsensResultValue cmtSetSyncOutSettings(const int32_t instance, const CmtSyncOutSettings settings, const CmtDeviceId deviceId);		//!< \brief Set the outbound synchronization settings of an MT device. \details This function sets the current outbound synchronization settings of the MT device (sync mode, skip factor, offset and pulse width). This function does not work for Xbus Masters and should not be used for sensors connected to an Xbus Master. \see cmtGetSyncOutSettings cmtGetSyncOutMode cmtSetSyncOutMode cmtGetSyncOutPulseWidth cmtSetSyncOutPulseWidth cmtGetSyncOutSkipFactor cmtSetSyncOutSkipFactor cmtGetSyncOutOffset cmtSetSyncOutOffset cmtGetSyncInSettings cmtSetSyncInSettings cmtGetSyncMode cmtSetSyncMode
CMT_DLL_API XsensResultValue cmtSetSyncOutMode (const int32_t instance, const uint16_t mode, const CmtDeviceId deviceId);		//!< \brief Set the outbound synchronization mode of an MT device. \details This function sets the current outbound synchronization mode of the MT device. This function does not work for Xbus Masters and should not be used for sensors connected to an Xbus Master. \see cmtGetSyncOutMode cmtSetSyncOutSettings cmtGetSyncOutSettings cmtGetSyncOutPulseWidth cmtSetSyncOutPulseWidth cmtGetSyncOutSkipFactor cmtSetSyncOutSkipFactor cmtGetSyncOutOffset cmtSetSyncOutOffset cmtGetSyncInSettings cmtSetSyncInSettings cmtGetSyncMode cmtSetSyncMode
CMT_DLL_API XsensResultValue cmtSetSyncOutPulseWidth (const int32_t instance, const uint32_t pulseWidth, const CmtDeviceId deviceId);		//!< \brief Set the outbound synchronization pulse width of an MT device. \details This function sets the current outbound synchronization pulse width of the MT device. This function does not work for Xbus Masters and should not be used for sensors connected to an Xbus Master. \see cmtGetSyncOutPulseWidth cmtSetSyncOutSettings cmtGetSyncOutSettings cmtSetSyncOutMode cmtGetSyncOutMode cmtGetSyncOutSkipFactor cmtSetSyncOutSkipFactor cmtGetSyncOutOffset cmtSetSyncOutOffset cmtGetSyncInSettings cmtSetSyncInSettings cmtGetSyncMode cmtSetSyncMode
CMT_DLL_API XsensResultValue cmtSetSyncOutSkipFactor (const int32_t instance, const uint16_t skipFactor, const CmtDeviceId deviceId);		//!< \brief Set the outbound synchronization skip factor of an MT device. \details This function sets the current outbound synchronization skip factor of the MT device. This function does not work for Xbus Masters and should not be used for sensors connected to an Xbus Master. \see cmtSetSyncOutSkipFactor cmtSetSyncOutSettings cmtGetSyncOutSettings cmtSetSyncOutMode cmtGetSyncOutMode cmtGetSyncOutPulseWidth cmtSetSyncOutPulseWidth cmtGetSyncOutOffset cmtSetSyncOutOffset cmtGetSyncInSettings cmtSetSyncInSettings cmtGetSyncMode cmtSetSyncMode
CMT_DLL_API XsensResultValue cmtSetSyncOutOffset (const int32_t instance, const uint32_t offset, const CmtDeviceId deviceId);		//!< \brief Set the outbound synchronization offset of an MT device. \details This function sets the current outbound synchronization offset of the MT device. This function does not work for Xbus Masters and should not be used for sensors connected to an Xbus Master. \see cmtGetSyncOutOffset cmtSetSyncOutSettings cmtGetSyncOutSettings cmtSetSyncOutMode cmtGetSyncOutMode cmtGetSyncOutPulseWidth cmtSetSyncOutPulseWidth cmtGetSyncOutSkipFactor cmtSetSyncOutSkipFactor cmtGetSyncInSettings cmtSetSyncInSettings cmtGetSyncMode cmtSetSyncMode
CMT_DLL_API XsensResultValue cmtSetSyncMode(const int32_t instance, const CmtXmSyncMode mode, const CmtDeviceId deviceId);						//!< \brief Set the sync mode of the Xbus Master. \details This function sets the current synchronization mode used by the specified Xbus Master. This function is not valid for MT devices. \see cmtGetSyncMode cmtGetSyncInSettings cmtSetSyncInSettings cmtGetSyncOutSettings cmtSetSyncOutSettings
CMT_DLL_API XsensResultValue cmtSetTimeoutConfig(const int32_t instance, uint32_t timeout = CMT4_DEFAULT_TIMEOUT_CONF);				//!< \brief Set the configuration mode timeout. \details When in config mode, the system uses a different message timeout than in measurement mode, since configuration messages can take longer to process than measurement mode messages. This function can be used to set the config mode timeout in ms. \see cmtGetTimeoutConfig cmtGetTimeoutMeasurement cmtSetTimeoutMeasurement cmtSetTimeoutData
CMT_DLL_API XsensResultValue cmtSetTimeoutData(const int32_t instance, uint32_t timeout = CMT4_DEFAULT_TIMEOUT_DATA);				//!< \brief Set the measurement mode data timeout. \details When in measurement mode, the system uses a different message timeout for data messages than for other messages. Data messages can be delayed sometimes due to wireless connectivity issues, so the data timeout is longer than the other timeout. When a data timeout occurs, CMT assumes that a serious error has occurred and will try to go to config mode. This function can be used to set the measurement mode data timeout in ms. \see cmtGetTimeoutMeasurement cmtSetTimeoutMeasurement cmtGetTimeoutConfig cmtSetTimeoutConfig
CMT_DLL_API XsensResultValue cmtSetTimeoutMeasurement(const int32_t instance, uint32_t timeout = CMT4_DEFAULT_TIMEOUT_MEAS);		//!< \brief Set the measurement mode timeout. \details When in measurement mode, the system uses a different message timeout than in config mode, since measurement mode messages should be faster to process than config mode messages. This function can be used to set the measurement mode timeout in ms. \see cmtGetTimeoutMeasurement cmtGetTimeoutConfig cmtSetTimeoutConfig cmtSetTimeoutData
CMT_DLL_API XsensResultValue cmtSetTransmissionDelay(const int32_t instance, const uint16_t delay, const CmtDeviceId deviceId);					//!< \brief Set the RS485 acknowledge transmission delay of the device. \details This function sets the RS485 acknowledge transmission delay of the device. The RS485 acknowledge transmission delay determines the minimal delay to response on request messages. See the low-level communication documentation for more details. \see cmtGetAckTransmissionDelay
CMT_DLL_API XsensResultValue cmtSetXmOutputMode(const int32_t instance, const uint8_t mode, const CmtDeviceId deviceId);					//!< \brief Set the dual-output mode of the XM. \details This function sets the dual mode output mode of the Xbus Master. The dual output mode of the Xbus Master defines whether it will send data on the serial connection (at the same baud rate as the bluetooth connection) when it is connected via Bluetooth. When set to 0, the serial communication is disabled, data is sent over the serial bus. \note When dual-mode is enabled, the Xbus Master will NOT receive commands on the serial bus. \see cmtGetXmDualOutputMode
CMT_DLL_API XsensResultValue cmtSetXmPowerOff(const int32_t instance, const CmtDeviceId deviceId);												//!< \brief Switch off one or all connected Xbus Masters. \details This function tells one or all connected Xbus Masters to power down. This differs from cmtSetBusPowerState, because this function actually powers down the Xbus Master itself, while cmtSetBusPowerState only powers down the bus controlled by the Xbus Master. After This function is called, the Xbus Master must be manually switched on to make it operational again. \see cmtSetBusPowerState
CMT_DLL_API XsensResultValue cmtStoreXkfState(const int32_t instance, const CmtDeviceId deviceId);												//!< \brief Store important components of the XKF filter state to non-volatile memory. \details This function allows you to store some critical components of the internal XKF filter state to non-volatile memory. The stored settings will be used to initialize the filter whenever the sensor is switched to Measurement mode. \note This function is only available in Config mode.
CMT_DLL_API XsensResultValue cmtTestSynchronization(const int32_t instance, CmtDeviceId* problemId);											//!< \brief Check if the connected devices are hardware-synchronized. \details This function tests if all connected devices are synchronized and running at the same speed. If the problemId parameter is non-NULL, the device ID of the first non-synchronized device is returned in it. If all devices are synchronized, the function will return XRV_OK.	This operation will take some time, since samples are requested from the devices to test the synchronization.
CMT_DLL_API XsensResultValue cmtUpdateCachedSensorInformation(const int32_t instance, const CmtDeviceId deviceId);								//!< \brief Update the cached sensor data. \details Upon connection, sensor information is cached on the host PC to minimize communication with the MT devices. Sometimes the local cache can become out-of-sync, especially when altering settings with custom messages. Calling this function will retrieve all necessary information from the connected devices so the local cache is up to date again.

CMT_DLL_API XsensResultValue cmtUploadScenario(const int32_t instance, const uint8_t scenarioType, const uint8_t position, const CmtDeviceId deviceId);	//!< \brief Update a scenario definition in a Motion Tracker. \details Use this function to upload a new version of a scenario or a whole new scenario definition into a connected Motion Tracker. The scenario must be one of those reported by cmtGetAvailableSoftwareScenarios, which returns available scenarios from the xsb file supplied to cmtLoadScenarios. \param scenarioType The type number of the scenario to upload. \param position The position where the scenario should be uploaded. The available positions are 0,1,2,3 and 4. When writing to a position that is currently used, the current scenario will be overwritten. \remarks When using the cmtRestoreFactoryDefaults function or when reverting to the default values from MT Manager, the scenario list is also reverted to the default list. \see cmtGetScenario cmtSetScenario cmtGetAvailableScenarios cmtGetAvailableSoftwareScenarios cmtGetSoftwareScenario cmtSetSoftwareScenario cmtLoadScenarios
CMT_DLL_API XsensResultValue cmtResetSoftwareFilters(const int32_t instance, const CmtDeviceId deviceId);			//!< \brief Reset the software filters. \details Use this function to re-initialize the software filters. Stored biases will be used, but stored filter state read from a log file will not be used. \see cmtSetSoftwareXkf3Filtering \see cmtSetSoftwareXkf6Filtering

/////////////////////////////////////////////////////////////////////////////////////////
//////////// data handling //////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

CMT_DLL_API XsensResultValue cmtDataFindDeviceId(const int32_t instance, const CmtDeviceId dev, int32_t* index, void* bundle=NULL);				//!< \brief Find the index in the idList that matches the given device id.
CMT_DLL_API XsensResultValue cmtDataGetRawAcc(const int32_t instance, CmtShortVector* data, const CmtDeviceId dev=0, void* bundle=NULL);		//!< \brief Return the Raw Accelerometer component of a data bundle.
CMT_DLL_API int32_t cmtDataContainsRawAcc(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);									//!< \brief Check if the data bundle contains Raw Accelerometer data.
CMT_DLL_API XsensResultValue cmtDataGetRawGyr(const int32_t instance, CmtShortVector* data, const CmtDeviceId dev=0, void* bundle=NULL);		//!< \brief Return the Raw Gyroscope component of a data bundle.
CMT_DLL_API int32_t cmtDataContainsRawGyr(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);									//!< \brief Check if the data bundle contains Raw Gyroscope data.
CMT_DLL_API XsensResultValue cmtDataGetRawMag(const int32_t instance, CmtShortVector* data, const CmtDeviceId dev=0, void* bundle=NULL);		//!< \brief Return the Raw Magnetometer component of a data bundle.
CMT_DLL_API int32_t cmtDataContainsRawMag(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);									//!< \brief Check if the data bundle contains Raw Magnetometer data.
CMT_DLL_API XsensResultValue cmtDataGetRawTemp(const int32_t instance, uint16_t* data, const CmtDeviceId dev=0, void* bundle=NULL);		//!< \brief Return the Raw Temperature component of a data bundle.
CMT_DLL_API int32_t cmtDataContainsRawTemp(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);									//!< \brief Check if the data bundle contains Raw Temperature data.
CMT_DLL_API XsensResultValue cmtDataGetRawData(const int32_t instance, CmtRawData* data, const CmtDeviceId dev=0, void* bundle=NULL);			//!< \brief Return the Raw Data component of a data bundle.
CMT_DLL_API int32_t cmtDataContainsRawData(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);									//!< \brief Check if the data bundle contains Raw Data.
CMT_DLL_API XsensResultValue cmtDataGetGpsPvtData(const int32_t instance, CmtGpsPvtData* data, const CmtDeviceId dev=0, void* bundle=NULL);	//!< \brief Return the GPS PVT Data component of a data bundle.
CMT_DLL_API int32_t cmtDataContainsGpsPvtData(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);								//!< \brief Check if the data bundle contains GPS PVT Data.
CMT_DLL_API XsensResultValue cmtDataGetRawPressureData(const int32_t instance, CmtRawPressureData* data, const CmtDeviceId dev=0, void* bundle=NULL);	//!< \brief Return the Raw Pressure Data component of a data bundle.
CMT_DLL_API int32_t cmtDataContainsRawPressureData(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);								//!< \brief Check if the data bundle contains Raw Pressure Data.
CMT_DLL_API XsensResultValue cmtDataGetTemp(const int32_t instance, double* data, const CmtDeviceId dev=0, void* bundle=NULL);					//!< \brief Return the Temperature component of a data bundle.
CMT_DLL_API int32_t cmtDataContainsTemp(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);										//!< \brief Check if the data bundle contains Temperature data.
CMT_DLL_API XsensResultValue cmtDataGetCalAcc(const int32_t instance, CmtVector* data, const CmtDeviceId dev=0, void* bundle=NULL);			//!< \brief Return the Calibrated Accelerometer component of a data bundle.
CMT_DLL_API int32_t cmtDataContainsCalAcc(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);									//!< \brief Check if the data bundle contains Calibrated Accelerometer data.
CMT_DLL_API XsensResultValue cmtDataGetCalGyr(const int32_t instance, CmtVector* data, const CmtDeviceId dev=0, void* bundle=NULL);			//!< \brief Return the Calibrated Gyroscope component of a data bundle.
CMT_DLL_API int32_t cmtDataContainsCalGyr(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);									//!< \brief Check if the data bundle contains Calibrated Gyroscope data
CMT_DLL_API XsensResultValue cmtDataGetCalMag(const int32_t instance, CmtVector* data, const CmtDeviceId dev=0, void* bundle=NULL);			//!< \brief Return the Calibrated Magnetometer component of a data bundle.
CMT_DLL_API int32_t cmtDataContainsCalMag(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);									//!< \brief Check if the data bundle contains Calibrated Magnetometer data.
CMT_DLL_API XsensResultValue cmtDataGetCalData(const int32_t instance, CmtCalData* data, const CmtDeviceId dev=0, void* bundle=NULL);			//!< \brief Return the Calibrated Data component of a data bundle.
CMT_DLL_API int32_t cmtDataContainsCalData(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);									//!< \brief Check if the data bundle contains Calibrated Data.
CMT_DLL_API XsensResultValue cmtDataGetOriQuat(const int32_t instance, CmtQuat* data, const CmtDeviceId dev=0, void* bundle=NULL);				//!< \brief Return the Orientation component of a data item as a Quaternion.
CMT_DLL_API int32_t cmtDataContainsOriQuat(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);									//!< \brief Check if the data bundle contains Quaternion Orientation data. \note This function may return false, while cmtDataGetOriQuat can still return correct data. This is because the CMT dll can convert the different orientation formats into the other formats as requested.
CMT_DLL_API XsensResultValue cmtDataGetOriEuler(const int32_t instance, CmtEuler* data, const CmtDeviceId dev=0, void* bundle=NULL);			//!< \brief Return the Orientation component of a data item as Euler angles.
CMT_DLL_API int32_t cmtDataContainsOriEuler(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);									//!< \brief Check if the data bundle contains Euler Orientation data. \note This function may return false, while cmtDataGetOriEuler can still return correct data. This is because the CMT dll can convert the different orientation formats into the other formats as requested.
CMT_DLL_API XsensResultValue cmtDataGetOriMatrix(const int32_t instance, CmtMatrix* data, const CmtDeviceId dev=0, void* bundle=NULL);			//!< \brief Return the Orientation component of a data item as an Orientation Matrix.
CMT_DLL_API int32_t cmtDataContainsOriMatrix(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);									//!< \brief Check if the data bundle contains Matrix Orientation data. \note This function may return false, while cmtDataGetOriMatrix can still return correct data. This is because the CMT dll can convert the different orientation formats into the other formats as requested.
CMT_DLL_API XsensResultValue cmtDataGetAnalogIn1(const int32_t instance, CmtAnalogInData* data, const CmtDeviceId dev=0, void* bundle=NULL);	//!< \brief Return the AnalogIn 1 component of a data item
CMT_DLL_API int32_t cmtDataContainsAnalogIn1(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);									//!< \brief Check if the data bundle contains AnalogIn 1 data
CMT_DLL_API XsensResultValue cmtDataGetAnalogIn2(const int32_t instance, CmtAnalogInData* data, const CmtDeviceId dev=0, void* bundle=NULL);	//!< \brief Return the AnalogIn 1 component of a data item
CMT_DLL_API int32_t cmtDataContainsAnalogIn2(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);									//!< \brief Check if the data bundle contains AnalogIn 1 data
CMT_DLL_API XsensResultValue cmtDataGetPositionLLA(const int32_t instance, CmtVector* data, const CmtDeviceId dev=0, void* bundle=NULL);		//!< \brief Return the Position LLA component of a data bundle. Requires the MTi-G sensor.
CMT_DLL_API int32_t cmtDataContainsPositionLLA(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);								//!< \brief Check if the data bundle contains Position LLA data.
CMT_DLL_API XsensResultValue cmtDataGetVelocity(const int32_t instance, CmtVector* data, const CmtDeviceId dev=0, void* bundle=NULL);			//!< \brief Return the Velocity NED/NWU component of a data bundle. \details Whether the velocity will be in North-East-Down or North-West-Up coordinates depends on the output settings. The default mode is North-West-Up.
CMT_DLL_API int32_t cmtDataContainsVelocity(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);									//!< \brief Check if the data bundle contains Velocity data.
CMT_DLL_API XsensResultValue cmtDataGetSampleCounter(const int32_t instance, uint16_t* data, const CmtDeviceId dev=0, void* bundle=NULL);//!< \brief Return the Sample Counter component of the data bundle
CMT_DLL_API int32_t cmtDataContainsSampleCounter(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);								//!< \brief Check if the data bundle contains Sample Counter.
CMT_DLL_API XsensResultValue cmtDataGetUtcTime(const int32_t instance, CmtUtcTime* data, const CmtDeviceId dev=0, void* bundle=NULL);	//!< \brief Return the UTC Time component of a data bundle.
CMT_DLL_API int32_t cmtDataContainsUtcTime(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);								//!< \brief Check if the data bundle contains UTC Time.
CMT_DLL_API XsensResultValue cmtDataGetRtc(const int32_t instance, CmtTimeStamp* data, const CmtDeviceId dev=0, void* bundle=NULL);			//!< \brief Return the Real Time Clock component of the data bundle
CMT_DLL_API int32_t cmtDataContainsRtc(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);										//!< \brief Check if the data bundle contains Real Time Clock data.
CMT_DLL_API XsensResultValue cmtDataGetAccG(const int32_t instance, CmtVector* data, const CmtDeviceId dev=0, void* bundle=NULL);				//!< \brief Return the XKF Acc-G component of the data bundle
CMT_DLL_API int32_t cmtDataContainsAccG(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);										//!< \brief Check if the data bundle contains XKF Acc-G.
CMT_DLL_API XsensResultValue cmtDataGetStatus(const int32_t instance, uint8_t* data, const CmtDeviceId dev=0, void* bundle=NULL);		//!< \brief Return the Status component of the data bundle
CMT_DLL_API int32_t cmtDataContainsStatus(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);									//!< \brief Check if the data bundle contains Status information.

CMT_DLL_API void* cmtCreatePermanentData(const int32_t instance, void* bundle);
CMT_DLL_API XsensResultValue cmtDestroyPermanentData(const int32_t instance, void* bundle);

CMT_DLL_API XsensResultValue cmtDataGetRawGpsData(const int32_t instance, CmtGpsPvtData* data, const CmtDeviceId dev=0, void* bundle=NULL);	//!< \brief Obsolete, use cmtDataGetGpsPvtData instead. This function will be removed in future versions of the MT SDK. \see cmtDataGetGpsPvtData
CMT_DLL_API int32_t cmtDataContainsRawGpsData(const int32_t instance, const CmtDeviceId dev=0, void* bundle=NULL);							//!< \brief Obsolete, use cmtDataContainsGpsPvtData instead. This function will be removed in future versions of the MT SDK. \see cmtDataContainsGpsPvtData

#endif	// CMTDLL_H
