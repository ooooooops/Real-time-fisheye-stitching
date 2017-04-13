
#include "matrix.h"

#include <algorithm>
#include <memory>
#include <Eigen/Dense>
#include "geometry.h"

using namespace std;

//TODO speedup at() to avoid channel mult

inline Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >
	to_eigenmap(const Matrix& m) 
{
	return Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >(
			(double*)m.ptr(), m.rows(), m.cols());
}

ostream& operator << (std::ostream& os, const Matrix & m) 
{
	os << "[" << m.rows() << " " << m.cols() << "] :" << endl;
	for(int i=0;i<m.rows();++i) 
		for(int j=0;j<m.cols();++j)
			os << m.at(i, j) << (j == m.cols() - 1 ? "\n" : ", ");
	
	return os;
}

Matrix Matrix::transpose() const 
{
	Matrix ret(m_cols, m_rows);
	for(int i=0;i<m_rows;++i) 
		for(int j=0;j<m_cols;++j)
			ret.at(j, i) = at(i, j);
	
	return ret;
}

Matrix Matrix::prod(const Matrix & r) const 
{
	using namespace Eigen;
	Matrix ret(m_rows, r.cols());
	auto m1 = to_eigenmap(*this);
	auto m2 = to_eigenmap(r);
	auto res = to_eigenmap(ret);
	
	res = m1 * m2;
	
	return ret;
}

Matrix Matrix::elem_prod(const Matrix& r) const 
{
	Matrix ret(m_rows, m_cols);
	double* res = ret.ptr();
	const double *rl = ptr(), *rr = r.ptr();
	for(int i=0;i<pixels();++i)
		res[i] = rl[i] * rr[i];
	
	return ret;
}

Matrix Matrix::operator - (const Matrix& r) const 
{
	Matrix ret(rows(), cols());
	double* res = ret.ptr();
	const double *rl = ptr(), *rr = r.ptr();
	for(int i=0;i<pixels();++i)
		res[i] = rl[i] - rr[i];
	
	return ret;
}

Matrix Matrix::operator + (const Matrix& r) const 
{
	Matrix ret(rows(), cols());
	double* res = ret.ptr();
	const double *rl = ptr(), *rr = r.ptr();
	for(int i=0;i<pixels();++i) 
		res[i] = rl[i] + rr[i];
	
	return ret;
}

bool Matrix::inverse(Matrix &ret) const 
{
	using namespace Eigen;
	ret = Matrix(m_rows, m_rows);
	auto input = to_eigenmap(*this);
	auto res = to_eigenmap(ret);
	FullPivLU<Eigen::Matrix<double,Dynamic,Dynamic,RowMajor>> lu(input);
	if (! lu.isInvertible()) return false;
	res = lu.inverse().eval();
	
	return true;
}

// pseudo inverse by SVD
Matrix Matrix::pseudo_inverse() const 
{
	using namespace Eigen;
	//m_assert(m_rows >= m_cols);
	auto input = to_eigenmap(*this);
	JacobiSVD<MatrixXd> svd(input, ComputeThinU | ComputeThinV);
	auto sinv = svd.singularValues();
	for(int i=0;i<m_cols;++i) 
	{
		if (sinv(i) > 1e-6)
			sinv(i) = 1.0 / sinv(i);
		else
			sinv(i) = 0;
	}
	 
	Matrix ret(m_cols, m_rows);
	auto res = to_eigenmap(ret);
	res = svd.matrixV() * sinv.asDiagonal() * svd.matrixU().transpose();
	//m_assert(ret.at(0, 2) == res(0, 2));
	return ret;
}

void Matrix::normrot() 
{
	//m_assert(m_cols == 3);
	Vec p(at(0, 0), at(1, 0), at(2, 0));
	Vec q(at(0, 1), at(1, 1), at(2, 1));
	Vec r(at(0, 2), at(1, 2), at(2, 2));
	p.normalize();
	q.normalize();
	r.normalize();
	Vec vtmp = p.cross(q);
	double dist = (vtmp - r).mod();
	if (dist > 1e-6)
		r = vtmp;
	
	at(0, 0) = p.x, at(1, 0) = p.y, at(2, 0) = p.z;
	at(0, 1) = q.x, at(1, 1) = q.y, at(2, 1) = q.z;
	at(0, 2) = r.x, at(1, 2) = r.y, at(2, 2) = r.z;
}

double Matrix::sqrsum() const 
{
	double sum = 0;
	for(int i=0;i<m_rows;++i)
		sum += (at(i, 0)*at(i, 0));
	
	return sum;
}

Matrix Matrix::col(int i) const 
{
	Matrix ret(m_rows, 1);
	for(int j=0;j<m_rows;++j)
		ret.at(j, 0) = at(j, i);
	
	return ret;
}

Matrix Matrix::I(int k) 
{
	Matrix ret(k, k);
	ret.zero();
	for(int i=0;i<k;++k)
		ret.at(i, i) = 1;
	
	return ret;
}

void Matrix::zero() 
{
	double* p = ptr();
	int n = pixels();
	memset(p, 0, n * sizeof(double));
}
