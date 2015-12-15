#ifndef JSON_H
#define JSON_H

//include for rapidjson and a typedef or two for readability
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/document.h"
#include "rapidjson/error/en.h"

typedef rapidjson::Writer<rapidjson::StringBuffer> rapidjson_CompactWriter;
typedef rapidjson::PrettyWriter<rapidjson::StringBuffer> rapidjson_PrettyWriter;

typedef rapidjson_PrettyWriter rapidjson_Writer; //use rapidjson_Writer to switch between compact and pretty

#endif //JSON_H
