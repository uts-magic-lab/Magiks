import math
import numpy as np

def univariate_linear_regression(X, Y):
    N = len(X)
    print N
    x      = np.array(X)
    y      = np.array(Y)
    xx     = x*x
    yy     = y*y
    xy     = x*y
    x_bar  = sum(x)/N
    y_bar  = sum(y)/N
    xy_bar = sum(xy)/N
    xx_bar = sum(xx)/N
    betta  = (xy_bar - x_bar*y_bar)/(xx_bar - x_bar*x_bar)
    intercept = y_bar - betta*x_bar
    return (intercept, betta)

def linear_regression(X, y):
    '''
    y must be a vector and X a matrix  
    (intercept, coefficients, residuals)
    '''
    m      = X.shape[1] # Number of columns of X (Number of data attributes)
    n      = X.shape[0] # Number of rows of X    (Number of data points)
    assert len(y) == n, "Error from statistics.linear_regression(): Dimension mismatch!"
    one    = rlib.rep(1.0, n)
    X      = rlib.cbind(one, X)
    b      = np.dot(vecmat.left_pseudo_inverse(X) , y)
    y_hat  = np.dot(X, b)
    e      = y - y_hat
    return (b[0], b[1:m], e)
