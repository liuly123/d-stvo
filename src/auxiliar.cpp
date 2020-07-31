#include <auxiliar.h>

#define PI std::acos(-1.0)

// 运动学函数
Matrix3d skew(Vector3d v){

    Matrix3d skew;

    skew(0,0) = 0; skew(1,1) = 0; skew(2,2) = 0;

    skew(0,1) = -v(2);
    skew(0,2) =  v(1);
    skew(1,2) = -v(0);

    skew(1,0) =  v(2);
    skew(2,0) = -v(1);
    skew(2,1) =  v(0);

    return skew;
}

Vector3d skewcoords(Matrix3d M){
    Vector3d skew;
    skew << M(2,1), M(0,2), M(1,0);
    return skew;
}

Matrix4d inverse_se3(Matrix4d T){
    Matrix4d Tinv = Matrix4d::Identity();
    Matrix3d R;
    Vector3d t;
    t = T.block(0,3,3,1);
    R = T.block(0,0,3,3);
    Tinv.block(0,0,3,3) =  R.transpose();
    Tinv.block(0,3,3,1) = -R.transpose() * t;
    return Tinv;
}

Matrix4d expmap_se3(Vector6d x){
    Matrix3d R, V, s, I = Matrix3d::Identity();
    Vector3d t, w;
    Matrix4d T = Matrix4d::Identity();
    w = x.tail(3);
    t = x.head(3);
    double theta = w.norm();
    if( theta < 0.000001 )
        R = I;
    else{
        s = skew(w)/theta;
        R = I + s * sin(theta) + s * s * (1.0f-cos(theta));
        V = I + s * (1.0f - cos(theta)) / theta + s * s * (theta - sin(theta)) / theta;
        t = V * t;
    }
    T.block(0,0,3,4) << R, t;
    return T;
}

Vector6d logmap_se3(Matrix4d T){
    Matrix3d R, Id3 = Matrix3d::Identity();
    Vector3d Vt, t, w;
    Matrix3d V = Matrix3d::Identity(), w_hat = Matrix3d::Zero();
    Vector6d x;
    Vt << T(0,3), T(1,3), T(2,3);
    w  << 0.f, 0.f, 0.f;
    R = T.block(0,0,3,3);
    double cosine = (R.trace() - 1.f)/2.f;
    if(cosine > 1.f)
        cosine = 1.f;
    else if (cosine < -1.f)
        cosine = -1.f;
    double sine = sqrt(1.0-cosine*cosine);
    if(sine > 1.f)
        sine = 1.f;
    else if (sine < -1.f)
        sine = -1.f;
    double theta  = acos(cosine);
    if( theta > 0.000001 ){
        w_hat = theta*(R-R.transpose())/(2.f*sine);
        w = skewcoords(w_hat);
        Matrix3d s;
        s = skew(w) / theta;
        V = Id3 + s * (1.f-cosine) / theta + s * s * (theta - sine) / theta;
    }
    t = V.inverse() * Vt;
    x.head(3) = t;
    x.tail(3) = w;
    return x;
}

Matrix6d adjoint_se3(Matrix4d T){
    Matrix6d AdjT = Matrix6d::Zero();
    Matrix3d R = T.block(0,0,3,3);
    AdjT.block(0,0,3,3) = R;
    AdjT.block(0,3,3,3) = skew( T.block(0,3,3,1) ) * R ;
    AdjT.block(3,3,3,3) = R;
    return AdjT;
}

Matrix6d uncTinv_se3(Matrix4d T, Matrix6d covT ){
    Matrix6d covTinv = Matrix6d::Zero();
    Matrix6d adjTinv;
    adjTinv = adjoint_se3( inverse_se3(T) );
    covTinv = adjTinv * covT * adjTinv.transpose();
    return covTinv;
}

Matrix6d unccomp_se3(Matrix4d T1, Matrix6d covT1, Matrix6d covTinc ){
    Matrix6d covT2 ; // covariance of T2 = T1 * inverse(Tinc)
    Matrix6d adjT1 = adjoint_se3(T1);
    covT2 = covT1 + adjT1 * covTinc * adjT1.transpose();
    return covT2;
}

bool is_finite(const MatrixXd x){
    return ((x - x).array() == (x - x).array()).all();
}

void vector_mean_stdv_mad( vector<double> residues, double &mean, double &stdv )
{

    mean = 0.f;
    stdv = 0.f;

    if( residues.size() != 0 )
    {
        // Return the standard deviation of vector with MAD estimation
        int n_samples = residues.size();
        vector<double> residues_ = residues;
        sort( residues.begin(),residues.end() );
        double median = residues[ n_samples/2 ];
        for( int i = 0; i < n_samples; i++)
            residues[i] = fabsf( residues[i] - median );
        sort( residues.begin(),residues.end() );
        stdv = 1.4826 * residues[ n_samples/2 ];

        // return the mean with only the best samples
        int k = 0;
        for( int i = 0; i < n_samples; i++)
            if( residues_[i] < 2.0 * stdv )
            {
                mean += residues_[i];
                k++;
            }


        if( k >= int( 0.2 * residues.size()) )
            mean /= double(k);
        else
        {
            k = 0;
            mean = 0.f;
            for( int i = 0; i < n_samples; i++)
            {
                mean += residues_[i];
                k++;
            }
            mean /= double(k);
        }
    }

}

double vector_stdv_mad( vector<double> residues)
{
    if( residues.size() != 0 )
    {
        // Return the standard deviation of vector with MAD estimation
        int n_samples = residues.size();
        sort( residues.begin(),residues.end() );
        double median = residues[ n_samples/2 ];
        for( int i = 0; i < n_samples; i++)
            residues[i] = fabsf( residues[i] - median );
        sort( residues.begin(),residues.end() );
        double MAD = residues[ n_samples/2 ];
        return 1.4826 * MAD;
    }
    else
        return 0.0;
}

double vector_mean_mad(vector<double> v, double stdv, double K)
{
    double sum = 0.0;
    int k = 0;
    for( int i = 0; i < v.size(); i++ )
        if( v[i] < K * stdv )
        {
            sum += v[i];
            k++;
        }
    if( k > 0 )
        return sum / k;
    else
        return sum;
}

double robustWeightCauchy( double norm_res )
{
    // Cauchy
    return 1.0 / ( 1.0 + norm_res * norm_res );

    // Smooth Truncated Parabola
    /*if( norm_res <= 1.0 )
        return 1.0 - norm_res * norm_res;
    else
        return 0.0;*/

    // Tukey's biweight
    /*if( norm_res <= 1.0 )
        return pow( 1.0 - norm_res*norm_res ,2.0);
    else
        return 0.0;*/

    // Huber loss function
    /*if( norm_res <= 1.0 )
        return 1.0;
    else
        return 1.0 / norm_res;*/

    // Welsch
    //return exp( - norm_res*norm_res );


}

// TODO: l-lk部分
Vector2d FootDrop(Vector2d px, double A, double B, double C)
{
    double fd_x = (B * B * px[0] - A * B * px[1] - A * C) / (A * A + B * B);
    double fd_y = (-A * B * px[0] + A * A * px[1] - B * C) / (A * A + B * B);
    return Vector2d(fd_x,fd_y);
}

double GetGrad(Mat &src, Point2i point, Vector2d dir) {
    int X = point.x;//float转Xnt
    int Y = point.y;
    double grad_x =
            +src.data[(X - 1) * src.step + Y - 1]
            + 2 * src.data[(X - 1) * src.step + Y]
            + src.data[(X - 1) * src.step + Y + 1]
            - src.data[(X + 1) * src.step + Y - 1]
            - 2 * src.data[(X + 1) * src.step + Y]
            - src.data[(X + 1) * src.step + Y + 1];
    double grad_y =
            +src.data[(X - 1) * src.step + Y + 1]
            + 2 * src.data[X * src.step + Y + 1]
            + src.data[(X + 1) * src.step + Y + 1]
            - src.data[(X - 1) * src.step + Y - 1]
            - 2 * src.data[X * src.step + Y - 1]
            - src.data[(X + 1) * src.step + Y - 1];
    return abs(grad_x * dir[0] + grad_y * dir[1]);
}





























