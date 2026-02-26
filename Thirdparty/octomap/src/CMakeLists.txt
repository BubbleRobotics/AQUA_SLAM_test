SET (octomath_SRCS
        ./math/Vector3.cpp
        ./math/Quaternion.cpp
        ./math/Pose6D.cpp
        )


ADD_LIBRARY( octomath SHARED ${octomath_SRCS})

SET (octomap_SRCS
  AbstractOcTree.cpp
  AbstractOccupancyOcTree.cpp
  Pointcloud.cpp
  ScanGraph.cpp
  CountingOcTree.cpp
  OcTree.cpp
  OcTreeNode.cpp
  OcTreeStamped.cpp
  ColorOcTree.cpp
  )

# dynamic and static libs, see CMake FAQ:
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../../lib)
ADD_LIBRARY( octomap SHARED ${octomap_SRCS})
target_link_libraries(octomap octomath)