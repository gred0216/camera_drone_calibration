import numpy as np
import time


# % % Function definitions

def logarithm_map(in_mat):
    cos_theta = (np.trace(in_mat)-1)/2
    sin_theta = np.sqrt(1-cos_theta ** 2)
    theta = np.arccos(cos_theta)
    ln_R = theta/(2*sin_theta)*(in_mat - in_mat.transpose())

    if np.isnan(ln_R).any():
        print("Something wrong")
    out = [ln_R[2, 1],
           ln_R[0, 2],
           ln_R[1, 0]]

    return out


def SkewSymmetricMatrix(x):
    if (isinstance(x, np.ndarray) and len(x.shape) >= 2):
        return np.array([[0, -x[2][0], x[1][0]],
                         [x[2][0], 0, -x[0][0]],
                         [-x[1][0], x[0][0], 0]])
    else:
        return np.array([[0, -x[2], x[1]],
                         [x[2], 0, -x[0]],
                         [-x[1], x[0], 0]])


def RandomRotation(max_angle_rad):

    unit_axis = np.random.rand(3, 1)-0.5
    unit_axis = unit_axis/np.linalg.norm(unit_axis)
    angle = np.random.rand() * max_angle_rad
    R = RotationFromUnitAxisAngle(unit_axis, angle)
    return R


def RotationFromUnitAxisAngle(unit_axis, angle):

    if angle == 0:
        R = np.eye(3)
    else:
        so3 = SkewSymmetricMatrix(unit_axis)
        R = np.eye(3)+so3*np.sin(angle)+so3@so3*(1-np.cos(angle))
    return R


def ProjectOntoSO3(M):
    [U, _, V] = np.linalg.svd(M)
    R = U @ V
    if np.linalg.det(R) < 0:
        V[:, 2] = -V[:, 2]
    R = U @ V
    return R


def isRotationMatrix(R):
    # square matrix test
    if R.ndim != 2 or R.shape[0] != R.shape[1]:
        return False
    should_be_identity = np.allclose(
        R.dot(R.T), np.identity(R.shape[0], np.float))
    should_be_one = np.allclose(np.linalg.det(R), 1)
    return should_be_identity and should_be_one


def GeodesicL1Mean(R_input, b_outlier_rejection, n_iterations, thr_convergence, R_true):

    #   1. Initialize

    n_samples = len(R_input)

    vectors_total = np.zeros((9, n_samples))
    for i in range(n_samples):
        vectors_total[:, i] = np.reshape(R_input[i].transpose(), 9)

    s = np.median(vectors_total, 1)
    s = np.reshape(s, [3, 3]).transpose()
    R = ProjectOntoSO3(s)

    for j in range(n_iterations):

        vs = np.zeros((3, n_samples))
        v_norms = np.zeros((1, n_samples))
        for i in range(n_samples):
            # if not isRotationMatrix(R_input[i]):
            #     print("R input is wrong", i)
            # if not isRotationMatrix(R.transpose()):
            #     print("R is wrong", i)
            v = logarithm_map(R_input[i]@R.transpose())
            v_norm = np.linalg.norm(v)
            vs[:, i] = v
            v_norms[:, i] = v_norm

        #  Compute the inlier threshold(if we reject outliers)
        thr = np.inf
        if b_outlier_rejection is True:
            sorted_v_norms = np.sort(v_norms)
            v_norm_firstQ = sorted_v_norms[0, np.int0(np.ceil(n_samples/4))]
            if (n_samples <= 50):
                thr = max(v_norm_firstQ, 1)

            else:
                thr = max(v_norm_firstQ, 0.5)

        step_num = 0
        step_den = 0

        for i in range(n_samples):
            v = vs[:, i]
            v_norm = v_norms[0][i]
            if (v_norm > thr):
                continue

            step_num = step_num + v/v_norm
            step_den = step_den + 1/v_norm

        delta = step_num / step_den
        delta_angle = np.linalg.norm(delta)
        delta_axis = delta / delta_angle

        R_delta = RotationFromUnitAxisAngle(delta_axis, delta_angle)
        R = R_delta @ R
        if (delta_angle < thr_convergence):
            break

    return R


def ChordalL1Mean(R_input, b_outlier_rejection, n_iterations, thr_convergence):
    '''
    Input:
        R_input: N by 3 by 3 array containing N rotation matrices (numpy ndarray, shape (n_sample, 3, 3) ) 
        b_outlier_rejection: if True, outlier will be rejected (bool)
        n_iterations: number of iterations (int)
        thr_convergence: degree of convergence threshold (float)
    Output:
        R: 3 by 3 array rotation matrix
    '''

    # % 1. Initialize
    n_samples = len(R_input)

    vectors_total = np.zeros((9, n_samples))
    for i in range(n_samples):
        vectors_total[:, i] = np.reshape(R_input[i].transpose(), 9)

    s = np.median(vectors_total, 1)

    # % 2. Optimize
    for j in range(n_iterations):
        total_s = np.tile(s, (n_samples, 1)).transpose()
        if (sum(sum(abs(vectors_total - total_s)) == 0) != 0):
            s = s+np.random.random((s, 1), 1)*0.001

        v_norms = np.zeros((1, n_samples))
        for i in range(n_samples):
            v = vectors_total[:, i] - s
            v_norm = np.linalg.norm(v)
            v_norms[:, i] = v_norm

        # % Compute the inlier threshold(if we reject outliers).
        thr = np.inf
        if b_outlier_rejection is True:
            # sorted_v_norms = sorted(v_norms)
            # v_norm_firstQ = sorted_v_norms(np.ceil(n_samples/4))
            sorted_v_norms = np.sort(v_norms)
            v_norm_firstQ = sorted_v_norms[0, np.int0(np.ceil(n_samples/4))]

            if (n_samples <= 50):
                thr = max(v_norm_firstQ, 1.356)
                #  2*sqrt(2)*sin(1/2) is approximately 1.356
            else:
                thr = max(v_norm_firstQ, 0.7)
                #  2*sqrt(2)*sin(0.5/2) is approximately 0.7

        step_num = 0
        step_den = 0

        for i in range(n_samples):
            # v = vs[:, i]
            v_norm = v_norms[0][i]
            if (v_norm > thr):
                continue
            step_num = step_num + vectors_total[:, i] / v_norm
            step_den = step_den + 1 / v_norm

        s_prev = s
        s = step_num/step_den

        update_medvec = s - s_prev
        if (np.linalg.norm(update_medvec) < thr_convergence):
            # print('break')
            break

    s = np.reshape(s, [3, 3]).transpose()
    R = ProjectOntoSO3(s)
    return R


def test():
    # Example: Average 100 rotations (50 inliers, 50 outliers)

    n_inliers = 50
    n_outliers = 50
    inlier_noise_level = 5  # deg
    R_true = RandomRotation(np.pi)

    if not isRotationMatrix(R_true):
        print("R True is wrong")

    # 1. Create input rotaions:
    n_samples = n_inliers + n_outliers
    # R_samples = cell(1, n_samples)
    R_samples = np.zeros((n_samples, 3, 3))

    for i in range(n_samples):

        if (i <= n_inliers):
            # % Inliers: perturb by 5 deg.

            axis_perturb = np.random.rand(3, 1) - 0.5
            axis_perturb = axis_perturb/np.linalg.norm(axis_perturb)
            angle_perturb = np.random.normal(0, inlier_noise_level/180 * np.pi)
            R_perturb = RotationFromUnitAxisAngle(axis_perturb, angle_perturb)
            R_samples[i] = R_perturb @ R_true
        else:
            # Outliers: completely random.
            R_samples[i] = RandomRotation(np.pi)

    # % 2-a. Average them using Hartley's L1 geodesic method
    # % (with our initialization and outlier rejection scheme):

    b_outlier_rejection = True
    n_iterations = 10
    thr_convergence = 0.001

    time_geodesic = time.time()
    R_geodesic = GeodesicL1Mean(
        R_samples, b_outlier_rejection, n_iterations, thr_convergence, R_true)
    time_geodesic = time.time() - time_geodesic

    if not isRotationMatrix(R_geodesic):
        print("R_geodesic is wrong")

    # 2-b. Average them using our approximate L1 chordal method
    # (with our initialization and outlier rejection shceme)

    b_outlier_rejection = True
    n_iterations = 10
    thr_convergence = 0.001
    time_chordal = time.time()
    R_chordal = ChordalL1Mean(R_samples, b_outlier_rejection,
                              n_iterations, thr_convergence)
    time_chordal = time.time() - time_chordal

    # 3. Evaluate the rotation error(deg):

    error_GeodesicL1Mean = np.abs(
        np.arccos((np.trace(R_true@R_geodesic.transpose())-1)/2) / np.pi * 180.0)
    error_ChordalL1Mean = np.abs(
        np.arccos((np.trace(R_true@R_chordal.transpose())-1)/2) / np.pi * 180.0)

    print(['Error (geodesic L1 mean) = ', (error_GeodesicL1Mean),
           ' deg, took ', (time_geodesic*1000), ' ms'])
    print(['Error (chordal L1 mean) = ', (error_ChordalL1Mean),
           ' deg, took ', (time_chordal*1000), ' ms'])
    print()


# test()
