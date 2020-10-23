import numpy as np
import carla
import transforms3d

def carla_vec2numpy(vec):
    return np.array([vec.x, vec.y, vec.z])


def transform2mat(t):
    #
    xyz = carla_vec2numpy(t.location)
    xyz = np.array([xyz[0], -xyz[1], xyz[2]])
    roll = -t.rotation.roll
    pitch = t.rotation.pitch    
    yaw = -t.rotation.yaw
    R = transforms3d.taitbryan.euler2mat( 
        np.deg2rad(yaw), np.deg2rad(pitch), np.deg2rad(roll))

    M = np.zeros((4, 4))
    M[0:3, 0:3] = R
    M[0:3, 3] = xyz
    M[3, 3] = 1

    return M


def mat2transform(M):

    pitch, roll, yaw = transforms3d.taitbryan.mat2euler(M[0:3, 0:3])
    roll = np.rad2deg(roll)
    pitch = np.rad2deg(pitch)
    yaw = np.rad2deg(yaw)

    T = carla.Transform(
        carla.Location(x=M[0, 3], y=-M[1, 3], z=M[2, 3]),
        carla.Rotation(pitch=pitch, yaw=-yaw, roll=-roll),
    )

    return T


def relative_transform(source, target):
    source_t = transform2mat(source)
    target_t = transform2mat(target)
    source_t_inv = np.linalg.inv(source_t)

    relative_transform_mat = np.dot(target_t, source_t_inv)

    relative_transform_transform = mat2transform(relative_transform_mat)

    return relative_transform_transform

def switch_CoordinateSystem(xyz):
    xyz[1] = -xyz[1]
    return xyz

def transform_location(loc, target_transform):

    loc = switch_CoordinateSystem(loc)
    T = transform2mat(target_transform)
    return switch_CoordinateSystem(np.dot(np.linalg.inv(T), loc))



def main():

    #Input transform A
    T_a = carla.Transform(
        carla.Location(1, 1, 0), carla.Rotation(pitch=0, yaw=90, roll=0)
    )

    # Location in left handed coordinates
    loc = switch_CoordinateSystem(np.array([1, 1, 0, 1]))

    # #Input transform B
    # T_b = carla.Transform(
    #     carla.Location(1, 1, 1), carla.Rotation(pitch=0, yaw=90, roll=0)
    # )

    # #Compute relative transform
    # T_a2b = relative_transform(T_a, T_b)

    #Convert to a 4x4 matrix
    T_a_mat = transform2mat(T_a)

    T_a_back = mat2transform(T_a_mat)

    #Transform T_a to T_b using numpy
    T_a_loc_np = switch_CoordinateSystem(np.dot(np.linalg.inv(T_a_mat), loc))

    # #Transform T_a to T_b using carla 
    # T_b_prime = T_a2b.transform(T_a.location)

    #Expect both outputs to be -1, 6, 2
    print("T_a_mat", T_a_mat) # [-0.99999987  5.99999998  1.99999998  1.] - ~Correc
    print("T_a_loc_np", T_a_loc_np) # [-0.99999987  5.99999998  1.99999998  1.] - ~Correct 
    print("", )


if __name__ == "__main__":
    main()