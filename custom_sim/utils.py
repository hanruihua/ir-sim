import numpy as np

def wrap_to_pi(angle):
    """
    Wraps an angle to the range [-pi, pi].
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi

def get_transform(state):
    """
    Returns the rotation and translation matrix for a 2D state (x, y, theta).
    rotation_matrix (2x2), translation_vector (2x1)
    """
    x, y, theta = state
    c = np.cos(theta)
    s = np.sin(theta)
    
    R = np.array([
        [c, -s],
        [s,  c]
    ])
    
    t = np.array([[x], [y]])
    
    return R, t

def distance(p1, p2):
    """
    Euclidean distance between two points (numpy arrays).
    """
    return np.linalg.norm(p1 - p2)
