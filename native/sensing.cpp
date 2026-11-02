// Complete-point ground reduction. No ROS, network, threads or actuator I/O.
// ABI 1 uses float64 arithmetic even when the input PointCloud2 is float32.
// Build without fast-math/reassociation/FMA. Numerically marginal frames are
// explicitly returned to the NumPy implementation; they are never sampled.
#include <algorithm>
#include <cfenv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <new>
#include <vector>

extern "C" {
struct M20SensingResult {
    std::uint32_t abi;
    std::int32_t reason;
    std::uint64_t input_points;
    double floor_height;
    double coeff[3];
    double condition;
};
std::uint32_t m20_sensing_abi_version() { return 1; }
std::size_t m20_sensing_result_size() { return sizeof(M20SensingResult); }
}

namespace {
constexpr double resolution=.2;
constexpr int extent=32, side=65, cells=side*side;
enum Reason { ok=0, unsupported=1, rounding_mode=2, radius_boundary=3,
    ground_boundary=4, bin_boundary=5, insufficient_ground=6,
    sample_boundary=7, narrow_sample=8, degenerate=9, slope_boundary=10,
    output_boundary=11, height_boundary=12, allocation_failed=13 };

int fallback(M20SensingResult* result,int reason) { result->reason=reason;return 1; }
bool near(double value,double target,double epsilon=1e-11) {
    return std::fabs(value-target)<=epsilon;
}
// Neumaier summation limits the covariance difference from NumPy's pairwise
// reduction. Classification thresholds have an explicit numerical fallback.
struct Sum {
    double value=0.,correction=0.;
    void add(double x) {
        const double total=value+x;
        correction+=(std::fabs(value)>=std::fabs(x))?(value-total)+x:(x-total)+value;
        value=total;
    }
    double get() const { return value+correction; }
};

template<typename T>
int reduce(const T* points,std::size_t count,std::uint8_t* floor,
           std::uint8_t* obstacle,double* bounds,M20SensingResult* result) {
    std::vector<std::uint32_t> valid,samples;
    valid.reserve(count);samples.reserve(count);
    std::uint64_t bins[16]={};std::size_t ground_count=0;
    // Pass 1: preserve every finite point in the original radial interval and
    // count the same rounded integer ground-height bins, lowest bin wins ties.
    for (std::size_t i=0;i<count;++i) {
        const double x=points[3*i],y=points[3*i+1],z=points[3*i+2];
        if (!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z))continue;
        const double radius=std::sqrt(x*x+y*y);
        if (near(radius,.55)||near(radius,6.))return fallback(result,radius_boundary);
        if (!(radius>.55&&radius<6.))continue;
        valid.push_back(static_cast<std::uint32_t>(i));
        if (near(z,-.9)||near(z,-.15))return fallback(result,ground_boundary);
        if (z>-.9&&z<-.15) {
            const double scaled=z/.05;
            if (near(std::fabs(scaled-std::floor(scaled)),.5,1e-10))return fallback(result,bin_boundary);
            const int bin=static_cast<int>(std::nearbyint(scaled));
            if (bin < -18||bin > -3)return fallback(result,unsupported);
            ++bins[bin+18];++ground_count;
        }
    }
    result->input_points=valid.size();
    if (ground_count<80)return fallback(result,insufficient_ground);
    int mode=0;
    for (int i=1;i<16;++i)if (bins[i]>bins[mode])mode=i;
    const double level=(mode-18)*.05;
    // Pass 2: one mean/extent reduction over exactly the same sample rows.
    // The sequential three-channel mean matches ndarray.mean(axis=0)'s row
    // order. No full Nx3 converted/ground/centered arrays are allocated.
    double sx=0.,sy=0.,sz=0.;
    double minx=std::numeric_limits<double>::infinity(),miny=minx,maxx=-minx,maxy=-minx;
    for (const auto i:valid) {
        const double x=points[3*i],y=points[3*i+1],z=points[3*i+2];
        if (!(z>-.9&&z<-.15))continue;
        if (near(std::fabs(z-level),.08))return fallback(result,sample_boundary);
        if (!(std::fabs(z-level)<.08))continue;
        samples.push_back(i);sx+=x;sy+=y;sz+=z;
        minx=std::min(minx,x);maxx=std::max(maxx,x);miny=std::min(miny,y);maxy=std::max(maxy,y);
    }
    if (samples.size()<80||maxx-minx<=1.2+1e-10||maxy-miny<=1.2+1e-10)
        return fallback(result,narrow_sample);
    const double mx=sx/samples.size(),my=sy/samples.size(),mz=sz/samples.size();
    // Pass 3: centered second moments, rather than raw-moment subtraction.
    Sum xx_,xy_,yy_,xz_,yz_;
    for (const auto i:samples) {
        const double x=static_cast<double>(points[3*i])-mx,y=static_cast<double>(points[3*i+1])-my,z=static_cast<double>(points[3*i+2])-mz;
        xx_.add(x*x);xy_.add(x*y);yy_.add(y*y);xz_.add(x*z);yz_.add(y*z);
    }
    const double xx=xx_.get(),xy=xy_.get(),yy=yy_.get(),xz=xz_.get(),yz=yz_.get();
    const double determinant=xx*yy-xy*xy;
    const double scale=std::max(1.,xx*yy);
    result->condition=determinant/scale;
    // NumPy owns degenerate/near-degenerate behavior and its original messages.
    if (!(determinant>1e-5*scale))return fallback(result,degenerate);
    const double ax=(xz*yy-yz*xy)/determinant,ay=(yz*xx-xz*xy)/determinant;
    const double az=mz-ax*mx-ay*my;
    const double slope=std::hypot(ax,ay),limit=std::tan(6.*3.14159265358979323846/180.);
    if (!std::isfinite(slope)||slope>=limit-1e-9)return fallback(result,slope_boundary);
    result->floor_height=az;result->coeff[0]=ax;result->coeff[1]=ay;result->coeff[2]=az;
    if (near(std::fabs(az*1000.-std::floor(az*1000.)),.5,1e-7))return fallback(result,output_boundary);
    std::fill(floor,floor+cells,0);std::fill(obstacle,obstacle+cells,0);
    for(int i=0;i<cells;++i) {
        bounds[4*i]=bounds[4*i+1]=std::numeric_limits<double>::infinity();
        bounds[4*i+2]=bounds[4*i+3]=-std::numeric_limits<double>::infinity();
    }
    // Pass 4: visit ALL filtered points, including isolated low objects and pits.
    // Only numerically ambiguous whole frames fall back; no point is skipped.
    const double margin=1e-8/std::max(.01,result->condition);
    for(const auto i:valid) {
        const double x=points[3*i],y=points[3*i+1],z=points[3*i+2];
        const double height=z-(x*ax+y*ay+az);
        if (near(std::fabs(height),.055,margin)||near(std::fabs(height),.12,margin)||near(height,1.5,margin))
            return fallback(result,height_boundary);
        const int gx=static_cast<int>(std::nearbyint(x/resolution));
        const int gy=static_cast<int>(std::nearbyint(y/resolution));
        const int id=(gx+extent)*side+(gy+extent);
        if (id<0||id>=cells)return fallback(result,unsupported);
        if (std::fabs(height)<.055)floor[id]=1;
        if ((height>.12&&height<1.5)||height<-.12) {
            obstacle[id]=1;double* b=bounds+4*id;
            b[0]=std::min(b[0],x);b[1]=std::min(b[1],y);b[2]=std::max(b[2],x);b[3]=std::max(b[3],y);
        }
    }
    result->reason=ok;return 0;
}
}

extern "C" int m20_sensing_reduce(const void* points,std::uint64_t count,
    std::uint32_t scalar_bytes,std::uint8_t* floor,std::uint8_t* obstacle,
    double* bounds,std::uint32_t capacity,M20SensingResult* result) {
    if (!result)return 1;
    *result={};result->abi=1;
    if (!points||!floor||!obstacle||!bounds||capacity<cells||count<100||count>10000000)
        return fallback(result,unsupported);
    if (std::fegetround()!=FE_TONEAREST)return fallback(result,rounding_mode);
    try {
        if (scalar_bytes==4)return reduce(static_cast<const float*>(points),count,floor,obstacle,bounds,result);
        if (scalar_bytes==8)return reduce(static_cast<const double*>(points),count,floor,obstacle,bounds,result);
    } catch(const std::bad_alloc&) { return fallback(result,allocation_failed); }
    catch(...) { return fallback(result,unsupported); }
    return fallback(result,unsupported);
}
