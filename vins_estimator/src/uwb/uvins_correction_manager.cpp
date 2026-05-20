#include "uvins_correction_manager.h"

UVINSCorrectionManager::UVINSCorrectionManager()
{
    clear();
}

void UVINSCorrectionManager::clear()
{
    for (int i = 0; i < UVINS_OPT_WINDOW_SIZE; ++i)
    {
        Us[i].setZero();
        dPs[i].setZero();
        Vps[i].setZero();
        Vqs[i].setIdentity();
        Headers[i] = 0.0;
        Ps_cov[i].setIdentity();
    }
}

bool UVINSCorrectionManager::isWindowReady() const
{
    return !Us[UVINS_OPT_WINDOW_SIZE - 1].isZero();
}

int UVINSCorrectionManager::validCount() const
{
    int count = 0;
    for (int i = 0; i < UVINS_OPT_WINDOW_SIZE; ++i)
    {
        if (!Us[i].isZero())
            ++count;
    }
    return count;
}

void UVINSCorrectionManager::updateState(double timestamp,
                                         const Eigen::Vector3d &aligned_uwb_ranges,
                                         const Eigen::Vector3d &vio_p,
                                         const Eigen::Quaterniond &vio_q,
                                         const Eigen::Vector3d &init_dP,
                                         const Eigen::Matrix3d &cov)
{
    int insert_index = lastValidIndex() + 1;
    if (insert_index >= UVINS_OPT_WINDOW_SIZE)
    {
        shiftLeft();
        insert_index = UVINS_OPT_WINDOW_SIZE - 1;
    }

    Us[insert_index] = aligned_uwb_ranges;
    dPs[insert_index] = init_dP;
    Vps[insert_index] = vio_p;
    Vqs[insert_index] = vio_q;
    Headers[insert_index] = timestamp;
    Ps_cov[insert_index] = cov;
}

int UVINSCorrectionManager::lastValidIndex() const
{
    for (int i = UVINS_OPT_WINDOW_SIZE - 1; i >= 0; --i)
    {
        if (!Us[i].isZero())
            return i;
    }
    return -1;
}

void UVINSCorrectionManager::shiftLeft()
{
    for (int i = 0; i < UVINS_OPT_WINDOW_SIZE - 1; ++i)
    {
        Us[i] = Us[i + 1];
        dPs[i] = dPs[i + 1];
        Vps[i] = Vps[i + 1];
        Vqs[i] = Vqs[i + 1];
        Headers[i] = Headers[i + 1];
        Ps_cov[i] = Ps_cov[i + 1];
    }

    Us[UVINS_OPT_WINDOW_SIZE - 1].setZero();
    dPs[UVINS_OPT_WINDOW_SIZE - 1].setZero();
    Vps[UVINS_OPT_WINDOW_SIZE - 1].setZero();
    Vqs[UVINS_OPT_WINDOW_SIZE - 1].setIdentity();
    Headers[UVINS_OPT_WINDOW_SIZE - 1] = 0.0;
    Ps_cov[UVINS_OPT_WINDOW_SIZE - 1].setIdentity();
}
