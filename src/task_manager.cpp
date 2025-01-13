#include <algorithm>
#include <climits>
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <plansys2_pddl_parser/Utils.h>

#include <memory>
#include <rclcpp/utilities.hpp>
#include <regex>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

using namespace std;

const int TIME_SLEEP = 10;

class task_manager : public rclcpp::Node 
{
public:
  atomic<bool> running = true;
  vector<std::string> visited_waypoints; // Ids of the waypoints as strings ex: "wp0"
  vector<uint> aruco_waypoints; // Ids of the aruco on the waypoints as uints ex: 12
  task_manager() : rclcpp::Node("task_manager"), state_(STARTING) {}


  /**
   * @brief Initializes all the objects needed to control the PDDL planning.
   *
   * This function creates shared pointers for the domain expert, planner client,
   * problem expert, and executor client. It also calls init_knowledge() to
   * initialize the problem knowledge.
   *
   * @return void
   */
  void init() {
    domain_expert_ = make_shared<plansys2::DomainExpertClient>();
    planner_client_ = make_shared<plansys2::PlannerClient>();
    problem_expert_ = make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = make_shared<plansys2::ExecutorClient>();
    init_knowledge();
  }



  /**
   * @brief Initializes the knowledge base for the planning problem.
   *
   * This function sets up the initial state of the planning problem by adding
   * instances of robots and waypoints, and defining predicates that describe
   * the initial state and goals of the system.
   *
   * It adds a robot instance, an 'unknown' waypoint, and four specific waypoints.
   * It also sets the initial position of the robot to the 'unknown' waypoint,
   * marks the 'unknown' waypoint as explored, and adds predicates indicating
   * that all specific waypoints need to be visited.
   *
   * This function does not take any parameters and does not return any value.
   * It operates by modifying the internal state of the problem_expert_ object.
   */
  void init_knowledge() 
  {
      problem_expert_->addInstance(plansys2::Instance{"rob", "robot"});
      problem_expert_->addInstance(plansys2::Instance{"unknown", "waypoint"});
      problem_expert_->addInstance(plansys2::Instance{"wp0", "waypoint"});
      problem_expert_->addInstance(plansys2::Instance{"wp1", "waypoint"});
      problem_expert_->addInstance(plansys2::Instance{"wp2", "waypoint"});
      problem_expert_->addInstance(plansys2::Instance{"wp3", "waypoint"});

      problem_expert_->addPredicate(plansys2::Predicate("(at-robby rob unknown)"));
      problem_expert_->addPredicate(plansys2::Predicate("(explored unknown)"));
      vecPreds_ = {
          plansys2::Predicate("(to_go wp0)"), plansys2::Predicate("(to_go wp1)"),
          plansys2::Predicate("(to_go wp2)"), plansys2::Predicate("(to_go wp3)")};
      for (const auto &pred : vecPreds_) 
      {
        problem_expert_->addPredicate(pred);
      }
  }


  /**
   * @brief Displays the progress of current actions being executed.
   *
   * This function retrieves feedback from the executor client and prints the progress
   * of various actions (move, explore_waypoint, move_to_min) to the console. It shows
   * the completion percentage for each action in progress.
   *
   * The function doesn't take any parameters and doesn't return any value. It operates
   * by side effects, printing the progress information to the standard output.
   *
   * After printing the progress, it moves the cursor up by the number of printed lines,
   * allowing for continuous updates in the same console area.
   */
  void show_progress() 
  {
    auto feedback = executor_client_->getFeedBack();
    if (feedback.action_execution_status.empty())
      return;
    for (const auto &action_feedback : feedback.action_execution_status) 
    {
      if (action_feedback.action == "move") 
      {
        cout << "Move to " << action_feedback.arguments[2]
             << " completed with percentage " << std::setprecision(3)
             << action_feedback.completion * 100.0 << "%          "
             << endl;
      } 
      else if (action_feedback.action == "explore_waypoint") 
      {
        cout << "Exploring " << action_feedback.arguments[1]
             << " completed with percentage " << std::setprecision(3)
             << action_feedback.completion * 100.0 << "%          "
             << endl;
      } 
      else if (action_feedback.action == "move_to_min") 
      {
        cout << "Moving to min " << action_feedback.arguments[2]
             << " completed with percentage " << std::setprecision(3)
             << action_feedback.completion * 100.0 << "%          "
             << endl;
      }
    }
    cout << string("\033[") 
          + to_string(feedback.action_execution_status.size()) 
          + "F";
  }

  /**
   * @brief  Retrieves the current progress from the executor client and updates
   *         visited waypoints and aruco_waypoints if a new waypoint was successfully explored.
   * 
   * The function queries feedback from `executor_client_` by calling `getFeedBack()`. 
   * It then iterates through the list of action execution statuses. Specifically, it looks for
   * actions of type `"explore_waypoint"`. If a matching action is found, it performs a regex match 
   * on the `message_status` field, extracting the ArUco marker ID (captured by the group `(\w+)` in the regex).
   * 
   * Once the ArUco ID is extracted:
   *  - It checks whether the waypoint has already been visited (`visited_waypoints`).
   *  - If not, inserts both the marker ID and the corresponding waypoint into two synchronized, 
   *    sorted containers (`aruco_waypoints` and `visited_waypoints`), ensuring consistent indexing
   *    by using `std::lower_bound` and `std::distance`.
   * 
   * @note    This function assumes that:
   *          - The waypoint name is accessible at `action_feedback.arguments[1]`.
   *          - The regex pattern `Id: (\w+)` is present in `action_feedback.message_status` for
   *            successfully explored waypoints.
   * 
   * @warning If the format of `message_status` changes, the regex extraction may fail. 
   *          Ensure that the `'Id: (\w+)'` pattern remains valid.
  */
  void get_progress() 
  {
    auto feedback = executor_client_->getFeedBack();
    smatch m;
    regex r(R"(Id: (\w+))");
    for (const auto &action_feedback : feedback.action_execution_status)
     {
      if (action_feedback.action == "explore_waypoint")
       {
        auto waypoint = action_feedback.arguments[1];
        // Regex_match returns true whether the regex matches the string and in
        // the 1 position of the matches(smatch) there is the first
        // capturing group.
        // Once there is a match it means that the the node has been explored
        // because the success string is printed.
        if (regex_match(action_feedback.message_status, m, r)) {
          if (find(visited_waypoints.begin(), visited_waypoints.end(), waypoint) == visited_waypoints.end())
           {
            // If the waypoint is not already inserted in the visited_waypoints
            // vector it means that is the first time that we reach it and we
            // have found the aruco id since the regex has matches so we can
            // work on it.

            // Here we take the id of the aruco
            auto toInsert = stoi(m[1].str());

            // Here we insert into the aruco_waypoints the id while keeping the
            // ascending ordering thanks to the lower_bound function
            auto iter = lower_bound(aruco_waypoints.begin(), aruco_waypoints.end(), toInsert);
            // *VERY* Important that the distance is checked *BEFORE* the
            // insertion otherwise the iter gets invalidated since the new value
            // is inserted before it
            auto dist = distance(aruco_waypoints.begin(), iter);
            aruco_waypoints.insert(iter, toInsert);

            // Since visited_waypoints and aruco_waypoints are vector of
            // different types we cannot use the same iterator. In order to have
            // the two arrays behave like a map of ordered elements we use the
            // index of where the new aruco id has been inserted thanks to the
            // distance function.
            //
            // Once we have the distance that is the index we can insert at that
            // position the waypoint string in the visited_waypoints vector.
            // next is the same as doing visited_waypoints.begin() + dist, but
            // safer
            visited_waypoints.insert(next(visited_waypoints.begin(), dist), waypoint);
          }
        }
      }
    }
  }

  /**
 * @brief  Advances the internal state machine by one step, driving the overall mission logic.
 *
 * The `step()` function is called periodically or on-demand to progress through the different
 * mission states. It begins by calling helper methods (`show_progress()` and `get_progress()`)
 * to retrieve status updates, then uses a switch-case structure on `state_` to determine
 * the next actions. Each case represents a distinct phase of the mission.
 *
 * **State Overview**:
 * - **STARTING**:
 *   - Sets the initial goal: `(and(explored wp0)(explored wp1)(explored wp2)(explored wp3))`
 *   - Retrieves and prints a plan (if found) using `domain_expert_`, `problem_expert_`, and `planner_client_`.
 *   - Starts plan execution if a valid plan exists, then transitions to `EXPLORE_WP`.
 *
 * - **EXPLORE_WP**:
 *   - Continues executing the current plan via `executor_client_->execute_and_check_plan()`.
 *   - On successful plan completion, checks if the global goal is satisfied:
 *     - If yes, transitions to `FINISHED_EXPLORING`.
 *     - If not, prints an error ("MAIN PLAN FAILED").
 *   - If plan execution fails, prints error messages and attempts to replan from the current state.
 *
 * - **FINISHED_EXPLORING**:
 *   - Clears space in the console for a fresh display of visited waypoints and associated ArUco IDs.
 *   - Prints out each visited waypoint alongside its corresponding marker ID.
 *   - Prepares a new goal by setting `(and(min <waypointName>))`, based on the earliest visited waypoint.
 *   - Retrieves and executes a new plan to move to the "smallest" (first visited) waypoint.
 *   - Transitions to `GO_TO_SMALLEST`.
 *
 * - **GO_TO_SMALLEST**:
 *   - Executes the plan to move to the smallest/first visited waypoint.
 *   - If successful and the new goal is satisfied, transitions to `FINISHED`.
 *   - If execution fails, attempts to replan.
 *
 * - **FINISHED**:
 *   - Prints a "Mission finished" message and halts further execution (`running = false`).
 *
 * @note    The function relies on several member variables:
 *          - `state_`: An enum or integer indicating the current mission state.
 *          - `executor_client_`, `planner_client_`, `problem_expert_`, `domain_expert_`: 
 *            Interfaces used for planning and execution in the PlanSys2 architecture.
 *          - `visited_waypoints`, `aruco_waypoints`: Synchronized lists tracking
 *            visited waypoints and their corresponding ArUco marker IDs.
 *          - `vecPreds_`: A collection of predicates used to determine new goals.
 *
 * @warning For the plan execution to succeed, ensure that:
 *          - Valid domain and problem definitions are set via `domain_expert_` and `problem_expert_`.
 *          - The waypoints listed in the goal (e.g., `wp0`, `wp1`, `wp2`, `wp3`) exist in the environment.
 *          - The PlanSys2 action servers are running and can handle the requested actions.
 *
 * @see     show_progress()
 * @see     get_progress()
 * @see     get_plan()
 */
  void step() 
  {
    show_progress();
    get_progress();

    switch (state_) 
    {
      case STARTING: 
      {
        // Set the initial goal
        problem_expert_->setGoal(plansys2::Goal("(and(explored wp0)(explored wp1)(explored wp2)(explored wp3))"));

        // Compute the plan
        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        auto plan = planner_client_->getPlan(domain, problem);

        // If the plan contains something
        if (plan.has_value()) 
        {
          cout << "Plan found:" << endl;
          for (const auto &action : plan.value().items) 
          {
            cout << action.action << " [";
          }
          cout << "Goal: "
               << parser::pddl::toString(problem_expert_->getGoal())
               << endl;
        }
        else if(!plan.has_value()) 
        {
          std::cout << "Could not find plan to reach goal "
                    << parser::pddl::toString(problem_expert_->getGoal())
                    << std::endl;
          break;
        }

        // Execute the plan
        if (executor_client_->start_plan_execution(plan.value())) 
        {
          state_ = EXPLORE_WP;
        }
      } break;

      case EXPLORE_WP: 
      {
        auto feedback = executor_client_->getFeedBack();
        if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) 
        {
          // Check which waypoint has been explored to add predicate
          if (executor_client_->getResult().value().success) 
          {
            std::cout << "Successful finished " << std::endl;

            // If the goal is satisfied then we can go in the finish state
            if (problem_expert_->isGoalSatisfied(problem_expert_->getGoal())) 
            {
              state_ = FINISHED_EXPLORING;
            } 
            else 
            {
              std::cout << "MAIN PLAN FAILED" << std::endl;
            }
          } 
          else 
          {
            for (const auto &action_feedback : feedback.action_execution_status) 
            {
              if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) 
              {
                std::cout << "[" << action_feedback.action
                          << "] finished with error: "
                          << action_feedback.message_status << std::endl;
              }
            }

            auto pe = problem_expert_->getPredicates();
            for (const auto &pred : pe) 
            {
              if (pred.name == "at-robby") 
              {
                problem_expert_->removePredicate(pred);
              }
            }
            problem_expert_->addPredicate(plansys2::Predicate("(at-robby rob unknown)"));
            // Replan
            auto domain = domain_expert_->getDomain();
            auto problem = problem_expert_->getProblem();
            auto plan = planner_client_->getPlan(domain, problem);

            if (!plan.has_value())
            {
              cout << "Unsuccessful replan attempt to reach goal "
                   << parser::pddl::toString(problem_expert_->getGoal())
                   << endl;
              break;
            }

            // Execute the plan
            executor_client_->start_plan_execution(plan.value());
          }
        }
      } break;

    case FINISHED_EXPLORING: 
    {
      // Get some space to not print over the previous showed data.
      cout << "\n\n\n\n\n\n\n\n\n\n\n";

      // Print the waypoints with the corresponding aruco node ids
      for (size_t i = 0; i < aruco_waypoints.size() - 1; i++) 
      {
        cout << aruco_waypoints[i] << "\t" << visited_waypoints[i] << endl;
      }
      cout << aruco_waypoints[aruco_waypoints.size() - 1] << "\t" << visited_waypoints[aruco_waypoints.size() - 1] << endl;

      auto pe = problem_expert_->getPredicates();
      for (const auto &pred : pe) 
      {
        if (pred.name == "at-robby") 
        {
          problem_expert_->removePredicate(pred);
        }
      }

      problem_expert_->addPredicate(plansys2::Predicate("(at-robby rob unknown)"));
      for (const auto &pred : vecPreds_) 
      {
        auto waypointName = pred.parameters[0].name;
        // "wp2"
        if (waypointName.find(visited_waypoints[0]) != std::string::npos) 
        {
          problem_expert_->removePredicate(pred);
          problem_expert_->setGoal(
              plansys2::Goal("(and(min " + waypointName + "))"));
          break;
        }
      }

      // Compute the plan
      auto domain = domain_expert_->getDomain();
      auto problem = problem_expert_->getProblem();
      auto plan = planner_client_->getPlan(domain, problem);

      cout << problem << endl;

      if (plan.has_value()) 
      {
        cout << "Second plan found:" << endl;
        for (const auto &action : plan.value().items) 
        {
          cout << action.action << " [";
        }
        cout << "Goal: "
             << parser::pddl::toString(problem_expert_->getGoal())
             << endl;
      }

      if (!plan.has_value()) 
      {
        cout << "Could not find plan to reach goal "
             << parser::pddl::toString(problem_expert_->getGoal())
             << endl;
        break;
      }

      // Execute the plan
      if (executor_client_->start_plan_execution(plan.value())) 
      {
        state_ = GO_TO_SMALLEST;
      }
    } break;

    case GO_TO_SMALLEST: 
    {
      auto feedback = executor_client_->getFeedBack();
      if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) 
      {
        // Check which waypoint has been explored to add predicate
        if (executor_client_->getResult().value().success) 
        {
          cout << "Successful finished " << endl;

          // Check if all goals are satisfied
          if (problem_expert_->isGoalSatisfied(problem_expert_->getGoal())) 
          { 
            state_ = FINISHED;
          }
          else 
          {
            cout << "MAIN PLAN FAILED" << endl;
          }
        } 
        else 
        {
          for (const auto &action_feedback : feedback.action_execution_status) 
          {
            if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) 
            {
              cout << "[" << action_feedback.action
                   << "] finished with error: "
                   << action_feedback.message_status << endl;
            }
          }

          // Replan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value()) 
          {
            std::cout << "Unsuccessful replan attempt to reach goal "
                      << parser::pddl::toString(problem_expert_->getGoal())
                      << std::endl;
            break;
          }

          // Execute the plan
          executor_client_->start_plan_execution(plan.value());
        }
      }

    } break;

    case FINISHED: 
    {
      cout << "Mission finished" << endl;
      running = false;
    }
    default:
      break;
    }
  }

private:
  std::vector<plansys2::Predicate> vecPreds_;
  typedef enum {
    STARTING,
    EXPLORE_WP,
    FINISHED_EXPLORING,
    GO_TO_SMALLEST,
    FINISHED
  } StateType;
  StateType state_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

void play_intro()
{
  cout<<"Hi!"<<endl;
  cout<<"We are Amir and Sayna"<<endl;
  cout<<"Welcome to the experimental robotics laboratory!"<<endl;
  cout<<"Please Wait ..."<<endl;

  for (int i = 0; i<TIME_SLEEP; i++)
  {
    cout<<"Please Wait "<< TIME_SLEEP - i << "seceonds!"<<endl;
    // Wait 10 seconds for the system to be ready
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  system("clear");
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<task_manager>();

  node->init();

  rclcpp::Rate rate(5);

  play_intro();

  while (node->running && rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
