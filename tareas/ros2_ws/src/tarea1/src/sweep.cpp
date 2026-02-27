#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <interfaces/action/execute_trajectory.hpp>
#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

class SweepNode : public rclcpp::Node
{
public:
  using ExecuteTrajectory = interfaces::action::ExecuteTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteTrajectory>;

  SweepNode() : Node("sweep_node")
  {
    param_client_ = std::make_shared<rclcpp::SyncParametersClient>(
        this, "/icc_controller");

    action_client_ = rclcpp_action::create_client<ExecuteTrajectory>(
        this, "/execute_trajectory");

    RCLCPP_INFO(this->get_logger(), "Esperando al servidor de parámetros...");
    while (!param_client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Servidor de parámetros no disponible...");
    }

    RCLCPP_INFO(this->get_logger(), "Esperando al servidor de acciones...");
    while (!action_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN(this->get_logger(), "Servidor de acciones no disponible...");
    }

    run_sweep();
  }

private:
  std::shared_ptr<rclcpp::SyncParametersClient> param_client_;
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr action_client_;

  void run_sweep()
  {
    for (int controller_type : {0, 1}) {

      RCLCPP_INFO(this->get_logger(), "Cambiando controller_type = %d", controller_type);
      param_client_->set_parameters({
        rclcpp::Parameter("controller_type", controller_type)
      });

      for (double kv = 0.1; kv <= 2.0 + 1e-6; kv += 0.1) {

        RCLCPP_INFO(this->get_logger(), "Nuevo kv = %.2f", kv);
        param_client_->set_parameters({
          rclcpp::Parameter("kv", kv)
        });

        std::cout << "\nPulsa ENTER cuando hayas reseteado el simulador...";
        std::cin.get();

        send_action();
      }
    }

    RCLCPP_INFO(this->get_logger(), "Barrido completo terminado.");
  }

  void send_action()
  {
    auto goal_msg = ExecuteTrajectory::Goal();
    goal_msg.target_pose.position.x = std::nan("");
    goal_msg.target_pose.position.y = std::nan("");
    goal_msg.target_pose.position.z = std::nan("");
    goal_msg.target_pose.orientation.x = 0.0;
    goal_msg.target_pose.orientation.y = 0.0;
    goal_msg.target_pose.orientation.z = 0.0;
    goal_msg.target_pose.orientation.w = 1.0;

    auto send_goal_options = rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions();
    send_goal_options.feedback_callback =
      [this](GoalHandle::SharedPtr,
             const std::shared_ptr<const ExecuteTrajectory::Feedback> feedback)
      {
        RCLCPP_INFO(this->get_logger(), "Feedback recibido");
      };

    auto future_goal_handle = action_client_->async_send_goal(goal_msg, send_goal_options);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Error enviando la acción");
      return;
    }

    auto goal_handle = future_goal_handle.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Meta rechazada");
      return;
    }

    auto result_future = action_client_->async_get_result(goal_handle);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future);

    auto result = result_future.get();
    RCLCPP_INFO(this->get_logger(), "Acción completada");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SweepNode>());
  rclcpp::shutdown();
  return 0;
}
