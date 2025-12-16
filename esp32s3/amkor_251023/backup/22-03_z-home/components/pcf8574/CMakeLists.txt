file(GLOB_RECURSE COMPONENT_SRC *.c)

idf_component_register(
	SRCS ${COMPONENT_SRC}
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver i2c_bus_manager
)

