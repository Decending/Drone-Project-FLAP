##          Copyright Emil Fresk 2016 - 2017
## Distributed under the Boost Software License, Version 1.0.
##    (See accompanying file LICENSE_1_0.txt or copy at
##          http://www.boost.org/LICENSE_1_0.txt)


########################################
# Add the example executable
########################################
add_executable(example example.cpp)

########################################
# Library linking
########################################
target_link_libraries(example kfly_comm)
