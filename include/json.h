#ifndef JSON_H
#define JSON_H

//include for rapidjson and a typedef or two for readability
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/document.h"
#include "rapidjson/error/en.h"


RAPIDJSON_NAMESPACE_BEGIN

class PrettyWriteFloat : public rapidjson::PrettyWriter<rapidjson::StringBuffer> {
public:
    typedef rapidjson::PrettyWriter<rapidjson::StringBuffer> Base;

    PrettyWriteFloat(rapidjson::StringBuffer& os) : Base(os) {}
    bool Float(float f) { PrettyPrefix(kNumberType); return WriteFloat(f); }

protected:
    bool WriteFloat(float f) {
        char buffer[25];
        int ret = snprintf(buffer, sizeof(buffer), "%g", f);
        for (int i = 0; i < ret; ++i)
            os_->Put(buffer[i]);
        return true;
    }
};

RAPIDJSON_NAMESPACE_END

//class rapidjson_Value : public rapidjson::Value {
//    float GetFloat() const { return (float)GetDouble(); }
//};

typedef rapidjson::Writer<rapidjson::StringBuffer> rapidjson_CompactWriter;
typedef rapidjson::PrettyWriteFloat rapidjson_PrettyWriterFloat;

typedef rapidjson_PrettyWriterFloat rapidjson_Writer; //use rapidjson_Writer to switch between compact and pretty

#endif //JSON_H
