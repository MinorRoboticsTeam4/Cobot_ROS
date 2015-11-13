## Workspace Layout

Following conventions from: [ros workspaces](http://wiki.ros.org/catkin/workspaces)

workspace_folder/         -- WORKSPACE
  src/                    -- SOURCE SPACE
    CMakeLists.txt        -- The 'toplevel' CMake file
    CHANGELOG.md	  -- Changelog 
    package_1/
      CMakeLists.txt
      package.xml
      ...
    package_n/
      CMakeLists.txt
      package.xml
      ...
  build/                  -- BUILD SPACE
    CATKIN_IGNORE         -- Keeps catkin from walking this directory
  devel/                  -- DEVELOPMENT SPACE (set by CATKIN_DEVEL_PREFIX)
    bin/
    etc/
    include/
    lib/
    share/
    .catkin
    env.bash
    setup.bash
    setup.sh
    ...
  install/                -- INSTALL SPACE (set by CMAKE_INSTALL_PREFIX)
    bin/
    etc/
    include/
    lib/
    share/
    .catkin 
    env.bash
    setup.bash
    setup.sh
    ...


## SRC packages layout

src packages (usually) follows this structure:
"package_name" is for additional readability if code needs to be seperate.


package_x /
	msg/					-- Contains Message(msg) types
	srv/					-- Contains Service(srv) types
	info/					-- Contains parameter(.yaml) files	
	launch/					-- Contains launch files

	include/package_name_x/ package_name_1	-- Contains C++ include headers	
	...
	include/package_name_x/ package_name_n	

	src/package_name_1			-- Contains source files
	...
	src/package_name_n	

	src/test/package_name_1			-- Contains gtest(C++ test) files
	...
	src/test/package_name_n	

	test/package_name_1			-- Contains ros test files for "package_name_"	
	...
	test/package_name_n

	CMakeLists.txt				-- Build file, (add src, include and test files for building)
	package.xml				-- General information of this package











