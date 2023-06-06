set(GIT_DIR_LOOKUP_POLICY ALLOW_LOOKING_ABOVE_CMAKE_SOURCE_DIR)
idf_build_get_property(target IDF_TARGET)

idf_component_register(
        INCLUDE_DIRS etl/include)
