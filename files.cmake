
set(header_include
)

# grouping
source_group("include" FILES ${header_include})

set(module_headers
    ${header_include}
)

set(source_src
	"${PROJECT_SOURCE_DIR}/src/main.hpp"
    "${PROJECT_SOURCE_DIR}/src/main.cpp"
)

set(source_hand
	"${PROJECT_SOURCE_DIR}/src/hand/hand_manager.hpp"
	"${PROJECT_SOURCE_DIR}/src/hand/hand_manager.cpp"
	"${PROJECT_SOURCE_DIR}/src/hand/hand_leap.cpp"
	"${PROJECT_SOURCE_DIR}/src/hand/hand_hand_mocap.cpp"
	"${PROJECT_SOURCE_DIR}/src/hand/hand_unist_mocap.cpp"
	"${PROJECT_SOURCE_DIR}/src/hand/hand_listener.cpp"
)

set(source_object
	"${PROJECT_SOURCE_DIR}/src/object/base_object.cpp"
	"${PROJECT_SOURCE_DIR}/src/object/soma_cube.cpp"
)

set(source_main_gui_imgui
    "${PROJECT_SOURCE_DIR}/src/main_gui/imgui/imgui_stl.cpp"
    "${PROJECT_SOURCE_DIR}/src/main_gui/imgui/imgui_stl.h"
)

set(source_main_gui
    "${PROJECT_SOURCE_DIR}/src/main_gui/main_gui.cpp"
    "${PROJECT_SOURCE_DIR}/src/main_gui/main_gui.hpp"
)

set(source_util
	"${PROJECT_SOURCE_DIR}/src/util/math.hpp"
	"${PROJECT_SOURCE_DIR}/src/util/math.cpp"
)

# grouping
source_group("src" FILES ${source_src})
source_group("src\\hand" FILES ${source_hand})
source_group("src\\object" FILES ${source_object})
source_group("src\\main_gui\\imgui" FILES ${source_main_gui_imgui})
source_group("src\\main_gui" FILES ${source_main_gui})
source_group("src\\util" FILES ${source_util})

set(module_sources
    ${source_src}
	${source_hand}
	${source_object}
	${source_main_gui_imgui}
	${source_main_gui}
	${source_util}
)
