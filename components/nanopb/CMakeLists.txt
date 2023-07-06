set(NANOPB_INCLUDE_DIR
        nanopb)

set(NANOPB_CPP_INCLUDE_DIR
        nanopb_cpp)

set(GEN_PROTOBUF_INCLUDE_DIR
        out)

idf_component_register(
        SRCS nanopb/pb_common.c
        nanopb/pb_decode.c
        nanopb/pb_encode.c
        out/spot.pb.c
        out/spot_config.pb.c
        out/wrapper.pb.c
        INCLUDE_DIRS ${NANOPB_INCLUDE_DIR}
        ${GEN_PROTOBUF_INCLUDE_DIR}
)