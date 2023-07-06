#!/usr/bin/env bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

#Usage: nanopb_generator.py [options] file.pb ...
#
#Options:
#  -h, --help            show this help message and exit
#  --version             Show version info and exit
#  -x FILE               Exclude file from generated #include list.
#  -e EXTENSION, --extension=EXTENSION
#                        Set extension to use instead of '.pb' for generated
#                        files. [default: .pb]
#  -H EXTENSION, --header-extension=EXTENSION
#                        Set extension to use for generated header files.
#                        [default: .h]
#  -S EXTENSION, --source-extension=EXTENSION
#                        Set extension to use for generated source files.
#                        [default: .c]
#  -f FILE, --options-file=FILE
#                        Set name of a separate generator options file.
#  -I DIR, --options-path=DIR, --proto-path=DIR
#                        Search path for .options and .proto files. Also
#                        determines relative paths for output directory
#                        structure.
#  --error-on-unmatched  Stop generation if there are unmatched fields in
#                        options file
#  --no-error-on-unmatched
#                        Continue generation if there are unmatched fields in
#                        options file (default)
#  -D OUTPUTDIR, --output-dir=OUTPUTDIR
#                        Output directory of .pb.h and .pb.c files
#  -Q FORMAT, --generated-include-format=FORMAT
#                        Set format string to use for including other .pb.h
#                        files. Value can be 'quote', 'bracket' or a format
#                        string. [default: #include "%s"]
#  -L FORMAT, --library-include-format=FORMAT
#                        Set format string to use for including the nanopb pb.h
#                        header. Value can be 'quote', 'bracket' or a format
#                        string. [default: #include <%s>]
#  --strip-path          Strip directory path from #included .pb.h file name
#  --no-strip-path       Opposite of --strip-path (default since 0.4.0)
#  --cpp-descriptors     Generate C++ descriptors to lookup by type (e.g.
#                        pb_field_t for a message)
#  -T, --no-timestamp    Don't add timestamp to .pb.h and .pb.c preambles
#                        (default since 0.4.0)
#  -t, --timestamp       Add timestamp to .pb.h and .pb.c preambles
#  -q, --quiet           Don't print anything except errors.
#  -v, --verbose         Print more information.
#  -s OPTION:VALUE       Set generator option (max_size, max_count etc.).
#  --protoc-opt=OPTION   Pass an option to protoc when compiling .proto files
#  --protoc-insertion-points
#                        Include insertion point comments in output for use by
#                        custom protoc plugins
#  -C, --c-style         Use C naming convention.
#
#Compile file.pb from file.proto by: 'protoc -ofile.pb file.proto'. Output will
#be written to file.pb.h and file.pb.c.

PROTO_DIR=$SCRIPT_DIR/proto
OUT_DIR=$SCRIPT_DIR/out
$SCRIPT_DIR/nanopb/generator/nanopb_generator.py -I $PROTO_DIR -D $OUT_DIR $PROTO_DIR/wrapper.proto
$SCRIPT_DIR/nanopb/generator/nanopb_generator.py -I $PROTO_DIR -D $OUT_DIR $PROTO_DIR/spot.proto
$SCRIPT_DIR/nanopb/generator/nanopb_generator.py -I $PROTO_DIR -D $OUT_DIR $PROTO_DIR/spot_config.proto
