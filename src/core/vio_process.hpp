#ifndef HSM_CORE_VIO_PROCESS_H
#define HSM_CORE_VIO_PROCESS_H

namespace hsm
{
    struct vio_process
    {
    public:
        vio_process()  = default;
        ~vio_process() = default;

        void process();

    };
    
} // namespace hsm


#endif