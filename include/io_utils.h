#ifndef IO_UTILS_H                                                            
#define IO_UTILS_H  

#include <google/protobuf/message.h>

#define PROTO_BIN_READ_LIMIT 600000000 /* 600MB */

using google::protobuf::Message;

/**
 * @brief LoadFromTextFile Loads the architecture of the net from 
 *                         .prototxt file which is plain text human readable.
 *
 * @param fpath Path to .prototxt file
 * @param msg   The generic type for Caffe.NetParameter
 *
 * @return true on success 
 */
bool LoadFromTextFile(const char *fpath, Message *msg);


/**
 * @brief LoadFromBinaryFile Loads the trained model with weights from
 *                           .caffemodel which is binary encoded.
 *
 * @param fpath Path to .prototxt file
 * @param msg   The generic type for Caffe.NetParameter
 *
 * @return 
 */
bool LoadFromBinaryFile(const char *fpath, Message *msg);

#endif /* IO_UTILS_H */
