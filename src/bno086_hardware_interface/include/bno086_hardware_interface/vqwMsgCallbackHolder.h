#include <functional>
#include <stdint.h>

#include "vqwPipe.h"

namespace vqw
{

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=
    //-=+~-=   vqwMsgCallbackHolder base class
    //-=+~-=

    class vqwMsgCallbackHolder_base
    {
      public:
        uint8_t      msgType                                      = 0;
        virtual void Dispatch(const ChannelDataMsgHeader_t *pCDM) = 0;
    };       //--- end of:  class vqwMsgCallbackHolder_base

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=
    //-=+~-=   vqwMsgCallbackHolder template
    //-=+~-=
    template<class T>
    class vqwMsgCallbackHolder : public vqwMsgCallbackHolder_base
    {
      public:
        using callback_t = std::function<void(const T *)>;

        vqwMsgCallbackHolder(callback_t cb) : callback(cb) { msgType = T::PRIVATE_DTO_ID; }

        virtual void Dispatch(const ChannelDataMsgHeader_t *pCDM)
        {
            T   msg;
            int rc = msg.deserialize(pCDM);
            if (rc == 0) callback(&msg);
        }

      private:
        callback_t callback;
    };       //--- end of:  class vqwMsgCallbackHolder_base

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

};       // namespace vqw