set(GIT_DIR_LOOKUP_POLICY ALLOW_LOOKING_ABOVE_CMAKE_SOURCE_DIR)
idf_build_get_property(target IDF_TARGET)

# depends on ETL
idf_component_register(
        SRCS src/rfsystem.cpp
        INCLUDE_DIRS inc
        REQUIRES etl)
component_compile_options(-std=gnu++17)
