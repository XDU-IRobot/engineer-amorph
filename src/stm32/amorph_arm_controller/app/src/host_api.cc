
#include "rpc/host_api.hpp"

namespace host {
std::shared_ptr<rpc_core::rpc> rpc{nullptr};

void Init(const std::shared_ptr<rpc_core::rpc> &rpc_instance) { rpc = rpc_instance; }
}  // namespace host