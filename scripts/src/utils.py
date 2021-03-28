import numpy as np



def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler[roll(x), pitch(y), raw(z)] in Quaternion[x, y, z, w]"""
    return np.array([ np.sin(roll*0.5) * np.cos(pitch*0.5) * np.cos(yaw*0.5) - np.cos(roll*0.5) * np.sin(pitch*0.5) * np.sin(yaw*0.5),
                      np.cos(roll*0.5) * np.sin(pitch*0.5) * np.cos(yaw*0.5) + np.sin(roll*0.5) * np.cos(pitch*0.5) * np.sin(yaw*0.5),
                      np.cos(roll*0.5) * np.cos(pitch*0.5) * np.sin(yaw*0.5) - np.sin(roll*0.5) * np.sin(pitch*0.5) * np.cos(yaw*0.5),
                      np.cos(roll*0.5) * np.cos(pitch*0.5) * np.cos(yaw*0.5) + np.sin(roll*0.5) * np.sin(pitch*0.5) * np.sin(yaw*0.5) ],
                    dtype=np.float64)


def euler_from_quaternion(x, y, z, w):
    """Convert Quaternion[x, y, z, w] in Euler[roll(x), pitch(y), raw(z)]"""
    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if (np.abs(sinp) >= 1):
        pitch = np.copysign(np.pi / 2, sinp) # use 90 degrees if out of range
    else:
        pitch = np.math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.math.atan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])


def quaternion_multiply(quaternion1, quaternion2):
    """Multiply two Quaternions[x, y, z, w]"""
    x1, y1, z1, w1 = quaternion1
    x2, y2, z2, w2 = quaternion2
    return np.array([ x1 * w2 + y1 * z2 - z1 * y2 + w1 * x2,
                       -x1 * z2 + y1 * w2 + z1 * x2 + w1 * y2,
                        x1 * y2 - y1 * x2 + z1 * w2 + w1 * z2,
                       -x1 * x2 - y1 * y2 - z1 * z2 + w1 * w2 ],
                    dtype=np.float64)


def quaternion_normalize(quaternion):
    """Normalize the quaternion[x, y, z, w]"""
    q = np.array(quaternion)
    return q / np.linalg.norm(q)


def quaternion_equals(quaternion1, quaternion2):
    """Compare two quaternions[x, y, z, w]"""
    q1 = np.array(quaternion1).reshape(1,4)
    q2 = np.array(quaternion2).reshape(4,1)
    d = np.dot(q1, q2)[0,0]
    if np.isclose(np.abs(d), 1.0, atol=0.001):
        return True
    return False


def quaternion_inverse(quaternion):
    """Quaternion[x, y, z, w] inverse"""
    x, y, z, w = quaternion
    return np.array((-x, -y, -z, w), dtype=np.float64)


def matrix_from_quaternion(quaternion):
    """Get rotation matrix from Quaternion[x, y, z, w] """
    x, y, z, w = quaternion
    return np.array([ [2*(w*w+x*x)-1,  2*(x*y-w*z),     2*(x*z+w*y)   ],
                      [2*(x*y+w*z),    2*(w*w+y*y)-1,   2*(y*z-w*x)   ],
                      [2*(x*z-w*y),    2*(y*z+w*x),     2*(w*w+z*z)-1 ]],
                    dtype=np.float64)


def quaternion_from_matrix(matrix):
    """Get quaternion[x, y, z, w] from rotation matrix"""
    tr = matrix[0,0] + matrix[1,1] + matrix[2,2]
    if (tr > 0):
        S = np.sqrt(tr+1.0) * 2
        qw = 0.25 * S
        qx = (matrix[2,1] - matrix[1,2]) / S
        qy = (matrix[0,2] - matrix[2,0]) / S
        qz = (matrix[1,0] - matrix[0,1]) / S
    elif ((matrix[0,0] > matrix[1,1]) & (matrix[0,0] > matrix[2,2])):
        S = np.sqrt(1.0 + matrix[0,0] - matrix[1,1] - matrix[2,2]) * 2
        qw = (matrix[2,1] - matrix[1,2]) / S
        qx = 0.25 * S
        qy = (matrix[0,1] + matrix[1,0]) / S
        qz = (matrix[0,2] + matrix[2,0]) / S
    elif (matrix[1,1] > matrix[2,2]):
        S = np.sqrt(1.0 + matrix[1,1] - matrix[0,0] - matrix[2,2]) * 2
        qw = (matrix[0,2] - matrix[2,0]) / S
        qx = (matrix[0,1] + matrix[1,0]) / S
        qy = 0.25 * S
        qz = (matrix[1,2] + matrix[2,1]) / S
    else:
        S = np.sqrt(1.0 + matrix[2,2] - matrix[0,0] - matrix[1,1]) * 2
        qw = (matrix[1,0] - matrix[0,1]) / S
        qx = (matrix[0,2] + matrix[2,0]) / S
        qy = (matrix[1,2] + matrix[2,1]) / S
        qz = 0.25 * S
    return np.array([qx, qy, qz, qw])
    

def homogeneous_matrix(trasformation):
    """Get homogeneous matrix from trasformation[px, py, pz,  qx, qy, qz, qw]"""
    translation  = np.array(trasformation[:3]).reshape(3,1)
    rotation = matrix_from_quaternion(trasformation[3:])
    padding = np.array([0, 0, 0, 1]).reshape(1,4)
    return np.append(np.append(rotation, translation, axis=1), padding, axis=0)


def homogeneous_matrix_inverse(matrix):
    """Homogeneous matrix[Rotation[x, y, z, w], Translation[x, y, z]] inverse"""
    rotation = matrix[:3, :3]
    translation = matrix[:3, [3]]
    padding = matrix[[3], :]
    inverted_rotation = rotation.transpose()
    new_translation = np.dot(-inverted_rotation, translation)
    return np.append(np.append(inverted_rotation, new_translation, axis=1), padding, axis=0)


def transform_from_matrix(matrix):
    """Get transform[px, py, pz,  qx, qy, qz, qw] from homogeneous matrix[Rotation[x, y, z, w], Translation[x, y, z]]"""
    rot_matrix = matrix[:3, :3]
    translation = matrix[:3, [3]].ravel()
    rotation = quaternion_from_matrix(rot_matrix).ravel()
    return np.concatenate((translation, rotation), axis=None)


def transform(pose1, pose2):
    """Get transform[px, py, pz,  qx, qy, qz, qw] from pose1 to pose2"""
    matrix1 = homogeneous_matrix(pose1)
    matrix2 = homogeneous_matrix(pose2)
    matrix3 = np.dot(matrix1, matrix2)
    return transform_from_matrix(matrix3)


def transform_inverse(pose):
    """Get transform[px, py, pz,  qx, qy, qz, qw] inverse"""
    matrix = homogeneous_matrix(pose)
    matrix_inv = homogeneous_matrix_inverse(matrix)
    return transform_from_matrix(matrix_inv)



if __name__ == "__main__":
    """TEST with python2"""
    import tf.transformations as tf

    test_enabled = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    # test_enabled = [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    num_test = 50
    tot_test = 0
    tot_successess = 0


    # quaternion_from_euler and euler_from_quaternion
    if test_enabled[0]:
        print("Test Conversions:")
        quaternion_successes = 0
        euler_successes = 0
        for i in range(num_test):
            # generate random
            q = tf.random_quaternion()
            # test euler_from_quaternion
            e1 = tf.euler_from_quaternion(q)
            e2 = euler_from_quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            quaternion_successes += int(np.allclose(e1, e2))
            # test quaternion_from_euler
            q1 = tf.quaternion_from_euler(e1[0], e1[1], e1[2])
            q2 = quaternion_from_euler(roll=e2[0], pitch=e2[1], yaw=e2[2])
            euler_successes += int(np.allclose(q1, q2))
        print('   -> Quaternion successes: {}/{}'.format(quaternion_successes, num_test))
        print('   -> Euler successes: {}/{}'.format(euler_successes, num_test))
        print("--------------------\n")
        tot_successess += quaternion_successes + euler_successes
        tot_test += num_test * 2


    # quaternion_multiply
    if test_enabled[1]:
        print("test quaternion_multiply")
        successes = 0
        for i in range(num_test):
            q1 = tf.random_quaternion()
            q2 = tf.random_quaternion()
            r1 = tf.quaternion_multiply(q1, q2)
            r2 = quaternion_multiply(q1, q2)
            successes += int(np.allclose(r1, r2))
        print('   -> successes: {}/{}'.format(successes, num_test))
        print("--------------------\n")
        tot_successess += successes
        tot_test += num_test

    
    # quaternion_normalize
    if test_enabled[2]:
        print("test quaternion_normalize")
        successes = 0
        for i in range(num_test):
            q0 = tf.random_quaternion() * 10

            q1 = quaternion_normalize(q0)
            n0 = np.linalg.norm(q1)

            successes += int(np.allclose(n0, 1))
        print('   -> successes: {}/{}'.format(successes, num_test))
        print("--------------------\n")
        tot_successess += successes
        tot_test += num_test


    # quaternion_equals
    if test_enabled[3]:
        print("test quaternion_equals")
        successes = 0

        q0 = np.array([ 0.707,  0.0,    0.0,    0.707])
        q1 = np.array([ 0.0,    0.707,  0.0,    0.707])
        q2 = np.array([ 0.0,    0.0,    0.707,  0.707])
        q3 = np.array([ 0.707,  0.0,    0.0,    0.707])
        q4 = np.array([-0.707,  0.0,    0.0,   -0.707])
        q5 = np.array([-0.707,  0.0,    0.0,    0.707])

        successes += int(quaternion_equals(q0, q1) == False)
        successes += int(quaternion_equals(q0, q2) == False)
        successes += int(quaternion_equals(q0, q3) == True)
        successes += int(quaternion_equals(q0, q4) == True)
        successes += int(quaternion_equals(q0, q5) == False)

        print('   -> successes: {}/{}'.format(successes, 5))
        print("--------------------\n")
        tot_successess += successes
        tot_test += 5


    # quaternion_inverse
    if test_enabled[4]:
        print("test quaternion_inverse")
        successes = 0
        for i in range(num_test):
            q = tf.random_quaternion()
            q1 = tf.quaternion_inverse(q)
            q2 = quaternion_inverse(q)
            successes += int(np.allclose(q1, q2))
        print('   -> successes: {}/{}'.format(successes, num_test))
        print("--------------------\n")
        tot_successess += successes
        tot_test += num_test


    # matrix_from_quaternion
    if test_enabled[5]:
        print("test matrix_from_quaternion")
        successes = 0
        for i in range(num_test):
            q = tf.random_quaternion()
            m1 = tf.quaternion_matrix(q)[:3, :3]
            m2 = matrix_from_quaternion(q)
            successes += int(np.allclose(m1, m2))
        print('   -> successes: {}/{}'.format(successes, num_test))
        print("--------------------\n")
        tot_successess += successes
        tot_test += num_test
    

    # quaternion_from_matrix
    if test_enabled[6]:
        print("test quaternion_from_matrix")
        successes_tf = 0
        successes_inv = 0
        for i in range(num_test):
            q0 = tf.random_quaternion()
            m0 = matrix_from_quaternion(q0)
            m1 = np.zeros(16).reshape(4,4)
            m1[:3, :3] = m0
            m1[3:4, :] = [0, 0, 0, 1]

            q1 = tf.quaternion_from_matrix(m1)
            q2 = quaternion_from_matrix(m0)

            successes_tf += int(quaternion_equals(q1, q2))
            successes_inv += int(quaternion_equals(q1, q0))
        print('   -> TF successes: {}/{}'.format(successes_tf, num_test))
        print('   -> Inverse successes: {}/{}'.format(successes_inv, num_test))
        print("--------------------\n")
        tot_successess += successes_tf + successes_inv
        tot_test += num_test * 2
    
    
    # homogeneous_matrix
    if test_enabled[7]:
        print("test homogeneous_matrix")
        successes = 0
        for i in range(num_test):
            q = tf.random_quaternion().tolist()
            p = [0, 0, 0]
            m1 = tf.quaternion_matrix(q)
            m2 = homogeneous_matrix(p + q)
            successes += int(np.allclose(m1, m2))
        print('   -> successes: {}/{}'.format(successes, num_test))
        print("--------------------\n")
        tot_successess += successes
        tot_test += num_test


    # homogeneous_matrix_inverse
    if test_enabled[8]:
        print("test homogeneous_matrix_inverse")
        successes = 0
        for i in range(num_test):
            m = tf.random_rotation_matrix()
            m[:3, 3:4] = tf.random_vector(3).reshape(3,1)
            m1 = tf.inverse_matrix(m)
            m2 = homogeneous_matrix_inverse(m)
            successes += int(np.allclose(m1, m2))
        print('   -> successes: {}/{}'.format(successes, num_test))
        print("--------------------\n")
        tot_successess += successes
        tot_test += num_test


    # transform_from_matrix
    if test_enabled[9]:
        print("test transform_from_matrix")
        successes = 0
        for i in range(num_test):
            p0 = tf.random_vector(3).tolist()
            q0 = tf.random_quaternion().tolist()

            m0 = homogeneous_matrix(p0 + q0)
            t0 = transform_from_matrix(m0)
            p1 = t0[:3]
            q1 = t0[3:]

            successes += int(np.allclose(p0, p1) and quaternion_equals(q0, q1))
        print('   -> successes: {}/{}'.format(successes, num_test))
        print("--------------------\n")
        tot_successess += successes
        tot_test += num_test


    # transform and transform_inverse
    if test_enabled[10]:
        print("test transform and transform_inverse")
        successes = 0
        for i in range(num_test):
            p0 = tf.random_vector(3)
            q0 = tf.random_quaternion()
            world_to_target = np.concatenate((p0, q0), axis=None)

            p1 = tf.random_vector(3)
            q1 = tf.random_quaternion()
            target_to_offset = np.concatenate((p1, q1), axis=None)
            offset_to_target = transform_inverse(target_to_offset)

            world_to_offset = transform(world_to_target, target_to_offset)
            world_to_target_new = transform(world_to_offset, offset_to_target)

            successes += int(np.allclose(world_to_target[:3], world_to_target_new[:3]) and \
                             quaternion_equals(world_to_target[3:], world_to_target_new[3:]))
        print('   -> successes: {}/{}'.format(successes, num_test))
        print("--------------------\n")
        tot_successess += successes
        tot_test += num_test

    
    # real case
    if test_enabled[11]:
        print("test real case")
        successes = 0
        wrist_to_tcp = [0.0, 0.0, 0.1035, 0.923879533, -0.382683432, 0.0, 0.0]
        tcp_to_wrist = [0.0, 0.0, 0.1035, -0.923879533, 0.382683432, -0.0, 0.0]
        world_to_tcp = [0.3, 0, 0.5, 0, 0, 0, 1]

        world_to_wrist = transform(world_to_tcp, tcp_to_wrist)
        world_to_wirst_correct = [0.3, 0.0, 0.6035, 0.923879532732408, -0.3826834321108402, 0.0, 0.0]

        successes += int(np.allclose(world_to_wrist[:3], world_to_wirst_correct[:3]) and \
                            quaternion_equals(world_to_wrist[3:], world_to_wirst_correct[3:]))
        print('   -> successes: {}/{}'.format(successes, 1))
        print("--------------------\n")
        tot_successess += successes
        tot_test += 1


    print("tot successess: {}/{}".format(tot_successess, tot_test))