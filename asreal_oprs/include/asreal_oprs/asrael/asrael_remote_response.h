#include <string>

enum AsraelRemoteCommandResponseCode {FALSE,TRUE,UNKNOWN,ERROR,UNDEFINED};

class AsrealRemoteCommandResponse
{
public:
    AsrealRemoteCommandResponse()
    {
            code_=UNDEFINED;
            message_="Response Message Undefined";
    }
    AsrealRemoteCommandResponse(AsraelRemoteCommandResponseCode code, std::string message)
    {
        code_=code;
        message_=message;
    }

    AsrealRemoteCommandResponse(xmlrpc_c::value result)
    {
        std::map<std::string, xmlrpc_c::value> const response((xmlrpc_c::value_struct(result)));

        try
        {
            message_ = (xmlrpc_c::value_string(response.at("message")));
            int code = (xmlrpc_c::value_int(response.at("code")));
            code_ = static_cast<AsraelRemoteCommandResponseCode>(code);
        }
        catch(...)
        {
            code_=ERROR;
            message_="Response Message Parsing Error";
        }
    }

    friend std::ostream & operator<<(std::ostream &os, AsrealRemoteCommandResponse const & response)
    {
        std::string code_str;
        switch (response.code_)
        {
            case FALSE: code_str = "FALSE"; break;
            case TRUE: code_str = "TRUE"; break;
            case ERROR: code_str = "ERROR"; break;
            case UNKNOWN: code_str = "UNNOWN"; break;
            default: code_str ="UNDEFINED";
        }

        os << "RESPONSE: code=" << code_str << ", message='" << response.message_ << "'";
        return os;
    }

    AsraelRemoteCommandResponseCode code_;
    std::string message_;

};

