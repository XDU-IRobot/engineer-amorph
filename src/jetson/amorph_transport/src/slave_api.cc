
#include "rpc/slave_api.hpp"

namespace slave {
std::shared_ptr<rpc_core::rpc> rpc{nullptr};

void Init(const std::shared_ptr<rpc_core::rpc> &rpc_instance) { rpc = rpc_instance; }
}  // namespace slave