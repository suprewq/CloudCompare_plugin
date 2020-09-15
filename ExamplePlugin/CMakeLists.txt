# CloudCompare example for standard plugins

# REPLACE ALL 'ExamplePlugin' OCCURENCES BY YOUR PLUGIN NAME
# AND ADAPT THE CODE BELOW TO YOUR OWN NEEDS!

#file(GLOB_RECURSE PCL_LIBRARIES "/usr/local/lib/*.so")
#file(GLOB_RECURSE VTK_LIBRARIES "/usr/lib/libvtk*.so")

#set(PCL_INCLUDE_DIRS "/usr/local/include/pcl-1.7")
find_package(PCL 1.7.2 REQUIRED)
add_library( ${PROJECT_NAME} STATIC )

# Add an option to CMake to control whether we build this plugin or not
option( PLUGIN_EXAMPLE_STANDARD "Install example plugin" ON )

if ( PLUGIN_EXAMPLE_STANDARD )
	project( ExamplePlugin )
	AddPlugin( NAME ${PROJECT_NAME} )
		
	add_subdirectory( include )
	add_subdirectory( src )
        add_subdirectory( ui )

        include_directories(
         include
          ${PCL_INCLUDE_DIRS}
        )

	target_include_directories( ${PROJECT_NAME}
         	PUBLIC
			${PCL_INCLUDE_DIRS}
	)


	# set dependencies to necessary libraries
	 target_link_libraries( ${PROJECT_NAME} 
                                CCPluginAPI
                                QCC_GL_LIB
                                ${PCL_LIBRARIES}
        )
endif()
