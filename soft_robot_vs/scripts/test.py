import dill
jacobian_binary_path="/home/krishnasathwik09/mer_lab/ros_ws/src/krishna_dr/soft_robot_vs/scripts/origami_2_module_Jacobian"

lambda_jacobian =  dill.load(open(jacobian_binary_path, "rb"))
print(lambda_jacobian(3,4,2,1,5,6,7))