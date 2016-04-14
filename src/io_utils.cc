#include <stdio.h>  /* perror() */
#include <fcntl.h>  /* open() */
#include <unistd.h> /* close() */

#include <iostream> /* cerr */
#include <fstream>  /* endl */
#include <string>   /* string */
#include <cstring>  /* c_str() */

#include "caffe.pb.h"
#include "Halide.h"
#include <google/protobuf/text_format.h>                /* Parse */
#include <google/protobuf/io/coded_stream.h>            /* CodedInputStream */
#include <google/protobuf/io/zero_copy_stream_impl.h>   /* FileInputStream */

#include "io_utils.h"

using namespace std;
using namespace Halide;
using namespace caffe;
using namespace google::protobuf;

using google::protobuf::TextFormat;
using google::protobuf::io::CodedInputStream;
//using google::protobuf::io::ZeroCopyInputStream;
using google::protobuf::io::FileInputStream;

bool
LoadFromTextFile(string fpath, Message *msg)
{
  int fd;

  if ((fd = open(fpath.c_str(), O_RDONLY)) == -1) {
    perror("Failed to open file");
    return false;
  }

  FileInputStream f_stream(fd);
  if (!TextFormat::Parse(&f_stream, msg)) {
    cerr << "Failed to parse" << endl;
    close(fd);
    return false;
  }

  close(fd);
  return true;
}

bool
LoadFromBinaryFile(string fpath, Message *msg)
{
  int fd;
  //ZeroCopyInputStream *raw_input;
  //CodedInputStream *coded_input;

  if ((fd = open(fpath.c_str(), O_RDONLY)) == -1) {
    perror("Failed to open file");
    return false;
  }

  FileInputStream raw_input(fd);
  CodedInputStream coded_input(&raw_input);

  /* fcn-32s-pascalcontext.caffemodel is 570MB. So need to set an approp
   * read limit so that protobuf doesn't throw an warning like this one:
   * "[libprotobuf WARNING google/protobuf/io/coded_stream.cc:537] Reading
   *  dangerously large protocol message.  If the message turns out to be larger
   *  than 597011289 bytes, parsing will be halted for security reasons.  To
   *  increase the limit (or to disable these warnings), see
   *  CodedInputStream::SetTotalBytesLimit() in
   *  google/protobuf/io/coded_stream.h." */
  coded_input.SetTotalBytesLimit(PROTO_BIN_READ_LIMIT, PROTO_BIN_READ_LIMIT);

  if (!msg->ParseFromCodedStream(&coded_input)) {
    cerr << "Failed to ParseFromCodedStream" << endl;
    close(fd);
    return false;
  }

  close(fd);
  return true;
}
