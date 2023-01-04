#ifndef _SWAP_BUFFER_CLASS_
#define _SWAP_BUFFER_CLASS_

#include <atomic>
#include <condition_variable>



namespace oculii
{
    template <class T>
    class SwapBuffer
    {
        public:
            SwapBuffer()
            {
                activePtr=&buffer[0];
                backPtr=&buffer[1];
                consumedData=true;
                producedData=false;
            }

            void Switch()
            {
                T* temp;
                temp=activePtr;
                activePtr=backPtr;
                backPtr=temp;
            }

            T* GetActivePtr()
            {
                return activePtr;
            }

            T* GetBackPtr()
            {
                return backPtr;
            }

            std::condition_variable cvProducedData;

            std::condition_variable cvConsumedData;

            std::atomic <bool> consumedData;

            std::atomic <bool> producedData;

        private:
            T buffer[2];
            T* activePtr;
            T* backPtr;      

    };
    

}; //end of namespace











#endif