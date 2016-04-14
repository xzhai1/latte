#ifndef IO_UTILS_H                                                            
#define IO_UTILS_H  
#include <string>

#include <google/protobuf/message.h> /* Message */

#define PROTO_BIN_READ_LIMIT 600000000 /* 600MB */

/**
 * @brief LoadFromTextFile Loads the architecture of the net from 
 *                         .prototxt file which is plain text human readable.
 *
 * @param fpath Path to .prototxt file
 * @param msg   The generic type for Caffe.NetParameter
 *
 * @return true on success 
 */
bool LoadFromTextFile(std::string fpath, google::protobuf::Message *msg);


/**
 * @brief LoadFromBinaryFile Loads the trained model with weights from
 *                           .caffemodel which is binary encoded.
 *
 * @param fpath Path to .caffemodel file
 * @param msg   The generic type for Caffe.NetParameter
 *
 * @return true on success
 */
bool LoadFromBinaryFile(std::string fpath, google::protobuf::Message *msg);

#endif /* IO_UTILS_H */
