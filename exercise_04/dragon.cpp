#include "utils/io.h"
#include "utils/points.h"

#include "ceres/ceres.h"
#include <math.h>


////////////////////////////////////////////////////////////////////////////////
////////////////////////Vladimir Yugay//////////////////////////////////////////
////////////////////////Jan Ruttinger///////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


struct RegistrationCostFunction
{
	RegistrationCostFunction(const Point2D& source_, const Point2D& target_, const Weight& w_)
		: source(source_), w(w_), target(target_)
	{
	}

	template<typename T>
	bool operator()(const T* const u, const T* const v, const T* const theta, T* residual) const
	{
		// TODO: Implement the cost function
		T a = cos(theta[0])* source.x - sin(theta[0])* source.y + u[0] - target.x;
		T b = sin(theta[0])* source.x + cos(theta[0])* source.y + v[0] - target.y;
		residual[0] = w.w * (pow(a, 2) + pow(b, 2));
		return true;
	}

private:
	const Point2D source;
	const Point2D target;
	const Weight w;
};



int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	// TODO: Read data points and the weights. Define the parameters of the problem
	const std::string file_path_1 = "../../data/exercise_4_data/points_dragon_1.txt";
	const std::string file_path_2 = "../../data/exercise_4_data/points_dragon_2.txt";
	const std::string file_path_weights = "../../data/exercise_4_data/weights_dragon.txt";

	const auto source = read_points_from_file<Point2D>(file_path_1);
	const auto target = read_points_from_file<Point2D>(file_path_2);
	const auto weights = read_points_from_file<Weight>(file_path_weights);

	double theta = 0.0;
	double u = 0.0;
	double v = 0.0;

	ceres::Problem problem;

	for (size_t i = 0; i < source.size(); ++i) {
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<RegistrationCostFunction, 1, 1, 1, 1> (
				new RegistrationCostFunction(source[i], target[i], weights[i])), 
			nullptr, &u, &v, &theta
		); 
	}

	ceres::Solver::Options options;
	options.max_num_iterations = 25;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;
	std::cout << "Final theta: " << theta*180.0/3.14 << "\tu: " << u << "\tv: " << v << std::endl;

	system("pause");
	return 0;
}