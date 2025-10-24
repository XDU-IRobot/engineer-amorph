
#pragma once

#include <rpc_core.hpp>

#include "typedefs.hpp"

namespace rpc {

class AmorphHost {
 public:
  void Init(std::shared_ptr<rpc_core::connection> &server_conn, std::shared_ptr<rpc_core::connection> &client_conn) {
    rpc_s_ = rpc_core::rpc::create(server_conn);
    rpc_c_ = rpc_core::rpc::create(client_conn);
    rpc_s_->set_ready(true);
    rpc_c_->set_ready(true);
  }

 private:
  std::shared_ptr<rpc_core::rpc> rpc_c_, rpc_s_;
};

}  // namespace rpc