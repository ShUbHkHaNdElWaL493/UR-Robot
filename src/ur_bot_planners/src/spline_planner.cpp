/*
    Shubh Khandelwal
*/

#include <iostream>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <vector>

struct Path
{
    Eigen::MatrixXd pos, vel, acc, jrk;
};

class SplineGenerator
{

    protected:

    int resolution;
    double epsilon;
    Eigen::Matrix4d basis_matrix;
    Eigen::MatrixXd t_pos, t_vel, t_acc, t_jrk;

    Path interpolate(const Eigen::VectorXd& p0, const Eigen::VectorXd& p1, const Eigen::VectorXd& p2, const Eigen::VectorXd& p3)
    {

        int dim = p0.size();
        Eigen::MatrixXd points(4, dim);
        points.row(0) = p0;
        points.row(1) = p1;
        points.row(2) = p2;
        points.row(3) = p3;

        Eigen::MatrixXd a = basis_matrix * points;
        Path res;
        res.pos = t_pos * a;
        res.vel = t_vel * a;
        res.acc = t_acc * a;
        res.jrk = t_jrk * a;
        
        return res;

    }

    public:

    SplineGenerator(const Eigen::Matrix4d& basis_matrix, int resolution, double epsilon) :
    basis_matrix(basis_matrix), resolution(resolution), epsilon(epsilon)
    {
        
        t_pos.resize(resolution, 4);
        t_vel.resize(resolution, 4);
        t_acc.resize(resolution, 4);
        t_jrk.resize(resolution, 4);

        for (int i = 0; i < resolution; ++i)
        {
            double t = (resolution > 1) ? static_cast<double>(i) / (resolution - 1) : 0.0;
            double t2 = t * t;
            double t3 = t2 * t;
            t_pos.row(i) << 1.0, t, t2, t3;
            t_vel.row(i) << 0.0, 1.0, 2.0 * t, 3.0 * t2;
            t_acc.row(i) << 0.0, 0.0, 2.0, 6.0 * t;
            t_jrk.row(i) << 0.0, 0.0, 0.0, 6.0;
        }
    
    }

    Path get_path(const std::vector<Eigen::VectorXd>& waypoints)
    {

        if (waypoints.empty())
        {
            return {};
        }

        std::vector<Eigen::VectorXd> clean_waypoints;
        clean_waypoints.reserve(waypoints.size() + 2);
        clean_waypoints.push_back(waypoints[0]);
        clean_waypoints.push_back(waypoints[0]);
        for (size_t i = 1; i < waypoints.size(); ++i)
        {
            if ((waypoints[i] - waypoints[i - 1]).norm() > epsilon)
            {
                clean_waypoints.push_back(waypoints[i]);
            }
        }
        clean_waypoints.push_back(clean_waypoints.back());
        int num_segments = clean_waypoints.size() - 3;
        if (num_segments <= 0)
        {
            return {};
        }

        int dim = waypoints[0].size();
        Path path;
        path.pos.resize(num_segments * resolution, dim);
        path.vel.resize(num_segments * resolution, dim);
        path.acc.resize(num_segments * resolution, dim);
        path.jrk.resize(num_segments * resolution, dim);

        for (int i = 0; i < num_segments; ++i)
        {
            Path res = interpolate(
                clean_waypoints[i], 
                clean_waypoints[i + 1], 
                clean_waypoints[i + 2], 
                clean_waypoints[i + 3]
            );
            int row_start = i * resolution;
            path.pos.block(row_start, 0, resolution, dim) = res.pos;
            path.vel.block(row_start, 0, resolution, dim) = res.vel;
            path.acc.block(row_start, 0, resolution, dim) = res.acc;
            path.jrk.block(row_start, 0, resolution, dim) = res.jrk;
        }

        return path;

    }

};

class BSplineGenerator : public SplineGenerator
{

    private:

    static Eigen::Matrix4d bs_basis_matrix()
    {
        Eigen::Matrix4d bs_bm;
        bs_bm <<  1.0/6.0,  4.0/6.0,  1.0/6.0,  0.0,
                 -3.0/6.0,  0.0,      3.0/6.0,  0.0,
                  3.0/6.0, -6.0/6.0,  3.0/6.0,  0.0,
                 -1.0/6.0,  3.0/6.0, -3.0/6.0,  1.0/6.0;
        return bs_bm;
    }

    public:

    BSplineGenerator(int resolution, double epsilon) : SplineGenerator(bs_basis_matrix(), resolution, epsilon)
    {}

};

class CatmullRomSplineGenerator : public SplineGenerator
{

    private:

    static Eigen::Matrix4d crs_basis_matrix()
    {
        Eigen::Matrix4d crs_bm;
        crs_bm <<  0.0,  1.0,  0.0,  0.0,
                  -0.5,  0.0,  0.5,  0.0,
                   1.0, -2.5,  2.0, -1.0,
                  -0.5,  1.5, -1.5,  0.5;
        return crs_bm;
    }

    public:

    CatmullRomSplineGenerator(int resolution, double epsilon) : SplineGenerator(crs_basis_matrix(), resolution, epsilon)
    {}

};

class SplinePlanner : public rclcpp::Node
{

    private:

    public:

    SplinePlanner() : Node("planner_node")
    {}

};

// class SplinePlanner(Node):

//     def __init__(self):

//         super().__init__("planner_node", allow_undeclared_parameters = True, automatically_declare_parameters_from_overrides = True)

//         self.conveyor_z = self.get_parameter("conveyor_z").get_parameter_value().double_value
//         self.case_length = self.get_parameter("case_length").get_parameter_value().double_value
//         self.case_width = self.get_parameter("case_width").get_parameter_value().double_value
//         self.case_height = self.get_parameter("case_height").get_parameter_value().double_value

//         self.crs = CatmullRomSplineGenerator(50)
//         self.bs = BSplineGenerator(50)

//         self.goals = []

//         self.homej = [
//             -1.5708,
//             -1.5708,
//             -1.5708,
//             -1.5708,
//             1.5708,
//             0
//         ]
//         self.home = {
//             "position" : np.array([
//                 0,
//                 1 + self.case_width / 2,
//                 self.conveyor_z + self.case_height + 0.5
//             ]),
//             "orientation" : False
//         }
//         self.pickup = {
//             "position" : np.array([
//                 0,
//                 1 + self.case_width / 2,
//                 self.conveyor_z + self.case_height
//             ]),
//             "orientation" : False
//         }
        
//         self.planner_service = self.create_service(Case, "/planner", self.planner_callback)

//     def planner_callback(self, request: Case.Request, response: Case.Response):
//         if request.execute:
//             self.execute()
//             response.success = True
//             return response
//         goal = {
//             "position" : np.array(request.position),
//             "orientation" : request.orientation
//         }
//         self.goals.append(goal)
//         response.success = True
//         return response

//     def execute(self):

//         pre_pickup = self.pickup["position"]
//         pre_pickup[2] += 0.1
//         self.waypoints = [self.home["position"]]
//         for i in range(len(self.goals)):
//             place = self.goals[i]["position"]
//             pre_place = place
//             pre_place[2] += 0.1
//             top = pre_pickup + pre_place / 2
//             top[2] += 0.6
//             self.waypoints.append(pre_pickup)
//             self.waypoints.append(self.pickup["position"])
//             self.waypoints.append(pre_pickup)
//             self.waypoints.append(top)
//             self.waypoints.append(pre_place)
//             self.waypoints.append(place)
//             self.waypoints.append(pre_place)
//             self.waypoints.append(top)
//         self.waypoints.append(self.home["position"])
        
//         pos, _, _, _ = self.crs.get_path(self.waypoints)
//         pos, vel, acc, jrk = self.bs.get_path(pos)
//         pos_x = [pos[i][0] for i in range(len(pos))]
//         pos_y = [pos[i][1] for i in range(len(pos))]
//         pos_z = [pos[i][2] for i in range(len(pos))]
//         vel_x = [vel[i][0] for i in range(len(vel))]
//         vel_y = [vel[i][1] for i in range(len(vel))]
//         vel_z = [vel[i][2] for i in range(len(vel))]
//         acc_x = [acc[i][0] for i in range(len(acc))]
//         acc_y = [acc[i][1] for i in range(len(acc))]
//         acc_z = [acc[i][2] for i in range(len(acc))]
//         jrk_x = [jrk[i][0] for i in range(len(jrk))]
//         jrk_y = [jrk[i][1] for i in range(len(jrk))]
//         jrk_z = [jrk[i][2] for i in range(len(jrk))]

//         fig, axs = plt.subplots(2, 2, figsize = (12, 10), subplot_kw = {"projection" : "3d"})
//         axs[0, 0].plot(pos_x, pos_y, pos_z, color = "red")
//         axs[0, 0].set_title("Position")
//         axs[0, 1].plot(vel_x, vel_y, vel_z, color = "blue")
//         axs[0, 1].set_title("Velocity")
//         axs[1, 0].plot(acc_x, acc_y, acc_z, color = "green")
//         axs[1, 0].set_title("Accelaration")
//         axs[1, 1].plot(jrk_x, jrk_y, jrk_z, color = "orange")
//         axs[1, 1].set_title("Jerk")

//         plt.tight_layout()
//         plt.show()

int main()
{
}