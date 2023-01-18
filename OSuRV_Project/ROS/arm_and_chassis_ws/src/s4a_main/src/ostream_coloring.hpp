/**
 * @file ostream_coloring.hpp
 * @date Feb 15, 2012
 *
 * @brief Coloring of output to ostream.
 * @version 1.1
 * @author Milos Subotic milos.subotic.sm@gmail.com
 *
 * @license LGPLv3
 *
 * @todo Implement Windows support
 */

#ifndef OSTREAM_COLORING_HPP_INCLUDED
#define OSTREAM_COLORING_HPP_INCLUDED

///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <string>

#ifdef WIN32
#include <windows.h>
#error "Not implemented on windows."
#endif

///////////////////////////////////////////////////////////////////////////////

namespace ostream_color_log{

	///////////////////////////////////////////////
	// Addition ostream manipulators for coloring ostream output.

	/// Add tab to std::ostream
	inline std::ostream& tab(std::ostream& os){
		return os.put('\t');
	}

	enum ostream_attributes{
		reset = '0', //everything
		bold, // bright
		dim,
		underline_v2, // didn't for me, but some worked for some
		underline, // worked for me
		blink,
		nothing, // in newer times same as blink
		reverse, // color scheme, not the text
		hidden
	};

	enum ostream_colors{
		black = '0', red, green, yellow, blue, magenta, cyan, white
	};

	struct _OstreamAttributeAndColorFormat{
		ostream_attributes attribute;
		ostream_colors foreground, background;
	};

	/**
	 * Manipulator builder for attributes and color of ostream.
	 * @param attribute attribute of ostream.
	 * @param foreground color of ostream.
	 * @param background color of ostream.
	 * @return manipulator.
	 */
	inline _OstreamAttributeAndColorFormat color(
            ostream_colors foreground = white,
            ostream_colors background = black,
            ostream_attributes attribute = nothing){
		_OstreamAttributeAndColorFormat format;
		format.attribute = attribute;
		format.foreground = foreground;
		format.background = background;
		return format;
	}

	///////////////////////////////
	// Interface for manipulators for coloring.


	/////////////////////////////
	// Manipulator interfaces for ostream. They are used for coloring console
	// on platform specific way.
#ifndef WIN32

	inline std::ostream& operator<<(std::ostream& os,
			ostream_colors foreground){
		os << "\033[3" << static_cast<char>(foreground) << "m";
		return os;
	}

	inline std::ostream& operator<<(std::ostream& os,
			ostream_attributes attribute){
		os << "\033[" << static_cast<char>(attribute) << "m";
		return os;
	}

	inline std::ostream& operator<<(std::ostream& os,
			_OstreamAttributeAndColorFormat format){
		os << "\033["
			<< static_cast<char>(format.attribute) << ";3"
			<< static_cast<char>(format.foreground) << ";4"
			<< static_cast<char>(format.background) << 'm';
		return os;
	}

#else
	/// @todo implement windows version
#error "Windows version is not implemented"
#endif

	/////////////////////////////
	// Manipulator interfaces for ofstream. They do nothing about coloring.

	inline std::ofstream& operator<<(std::ofstream& os,
			ostream_colors foreground){
		return os;
	}

	inline std::ofstream& operator<<(std::ofstream& os,
			ostream_attributes attribute){
		return os;
	}

	inline std::ofstream& operator<<(std::ofstream& os,
			_OstreamAttributeAndColorFormat format){
		return os;
	}


	////////////////////////////////////

} // namespace ostream_color_log{

///////////////////////////////////////////////////////////////////////////////

#endif // OSTREAM_COLORING_HPP_INCLUDED
