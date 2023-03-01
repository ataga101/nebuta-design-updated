#ifndef MESH_QUALITY_H
#define MESH_QUALITY_H

#include <math.h>

#include <wrap/igl/arap_parametrization.h>
#include <wrap/igl/lscm_parametrization.h>
#include <vcg/complex/algorithms/parametrization/distortion.h>
#include <vcg/space/outline2_packer.h>
#include <vcg/complex/algorithms/parametrization/uv_utils.h>
#include <vcg/complex/algorithms/mesh_to_matrix.h>
#include <vcg/complex/algorithms/parametrization/uv_utils.h>

#include <vcg/complex/algorithms/isotropic_remeshing.h>
#include <vcg/complex/algorithms/crease_cut.h>
#include <vcg/complex/algorithms/local_optimization/tri_edge_collapse.h>
#include <vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h>
#include <igl/gaussian_curvature.h>
#include <igl/doublearea.h>

#include "gauss_image_thinning.h"
#include "triangle_strips.h"
#include <igl/hausdorff.h>

enum ParamMode{PMConformal,PMArap,PMGaussianCurvature,PMGaussImageThinness, PMHausdorf};

template <class MeshType>
class MeshQuality
{
    typedef typename MeshType::VertexType  VertexType;
    typedef typename MeshType::ScalarType  ScalarType;
    typedef typename MeshType::FacePointer FacePointer;


public:

    static ParamMode &UVMode()
    {
        static ParamMode CurrUVMode=PMConformal;
        return CurrUVMode;
    }

    //{PMConformal,PMArap,PMCloth};

    static ScalarType & MinQ()
    {
        static ScalarType MinV=-0.05;
        return MinV;
    }

    static ScalarType & MaxQ()
    {
        static ScalarType MaxV=0.05;
        return MaxV;
    }

    static ScalarType & MaxSumOfGaussianCurvature()
    {
        static ScalarType MaxV=0.045;
        return MaxV;
    }

    static ScalarType & MaxGaussImageThickness()
    {
        static ScalarType MaxV = 0.00075;
        return MaxV;
    }

    static ScalarType & AreaScale()
    {
        static ScalarType MaxV = 0.1;
        return MaxV;
    }

    static ScalarType & MaxHausdorff()
    {
        static ScalarType MaxV = 0.02 / std::sqrt(AreaScale());
        return MaxV;
    }

    static bool & RemeshOnTest()
    {
        static bool rem=false;
        return rem;
    }

    static bool &ContinuousCheckSelfInt()
    {
        static bool check=false;
        return check;
    }

    MeshQuality(){ContinuousCheckSelfInt()=true;}

    ScalarType operator()(MeshType &m) const
    {

        #ifdef PRINT_PARAFASHION_TIMING
        steady_clock::time_point pre_param = steady_clock::now();
        #endif

        #ifdef COUNT_PARAM_CALLS
        param_calls_count ++;
        #endif
        /*
        if (RemeshOnTest())
            Remesh(m);
        */
        //RemeshByDeci(m);

        //            size_t numH=vcg::tri::Clean<TriMeshType>::CountHoles(m);
        //            if (numH!=1)
        //            {
        //                vcg::tri::io::ExporterPLY<TriMeshType>::Save(m,"test_holes.ply");
        //                assert(0);
        //            }

        //assert(numH==1);

        if (UVMode()==PMConformal)
        {
            vcg::tri::InitializeArapWithLSCM(m,0);
            #ifdef PRINT_PARAFASHION_TIMING
            steady_clock::time_point post_param = steady_clock::now();
            int param_time = duration_cast<microseconds>(post_param - pre_param).count();
            std::cout << "Param time : " << param_time << " [µs]" << std::endl;
            #endif
            vcg::tri::Distortion<TraceMesh,false>::SetQasDistorsion(m,vcg::tri::Distortion<TraceMesh,false>::EdgeComprStretch);
            ScalarType A=0;
            for (size_t i=0;i<m.face.size();i++)
            {
                if (m.face[i].Q()<MinQ())A+=vcg::DoubleArea(m.face[i]);
                if (m.face[i].Q()>MaxQ())A+=vcg::DoubleArea(m.face[i]);
            }
            //std::cout<<"Area:"<<A<<std::endl;
            return A;
        }

        if (UVMode()==PMArap)
        {
            vcg::tri::InitializeArapWithLSCM(m,0);
            //vcg::tri::OptimizeUV_ARAP(m,5,0,true);
#ifdef PRINT_PARAFASHION_TIMING
            steady_clock::time_point post_param = steady_clock::now();
            int param_time = duration_cast<microseconds>(post_param - pre_param).count();
            std::cout << "Param time : " << param_time << " [µs]" << std::endl;
#endif
            vcg::tri::Distortion<TraceMesh,false>::SetQasDistorsion(m,vcg::tri::Distortion<TraceMesh,false>::EdgeComprStretch);
            ScalarType A=0;
            for (size_t i=0;i<m.face.size();i++)
            {
                if (m.face[i].Q()<(MinQ()))A+=vcg::DoubleArea(m.face[i]);
                if (m.face[i].Q()>MaxQ())A+=vcg::DoubleArea(m.face[i]);
            }
            return A;
        }

        if (UVMode() == PMGaussianCurvature)
        {
            Eigen::MatrixXd V;
            Eigen::MatrixXi F;
            Eigen::VectorXd K;
            Eigen::VectorXd dblA;

            vcg::tri::MeshToMatrix< MeshType >::GetTriMeshData( m, F, V );
            igl::gaussian_curvature(V,F,K);
            igl::doublearea(V,F,dblA);

            K = K.cwiseAbs();

            double SumOfGaussianCurvature = 0;

            for(int f = 0; f < F.rows();f++)
            {
                for(int c = 0; c < 3;c++)
                    {
                        SumOfGaussianCurvature += dblA(f) * K(F(f, c));
                    }
            }

            SumOfGaussianCurvature /= AreaScale();

            //std::cerr << "Sum of gaussian curvature:" << SumOfGaussianCurvature << std::endl;

            return std::max(0., SumOfGaussianCurvature-MaxSumOfGaussianCurvature());
        }

        if(UVMode() == PMGaussImageThinness){
            Eigen::MatrixXd V, N, N2;
            Eigen::MatrixXi F;
            Eigen::VectorXd dblA;

            vcg::tri::MeshToMatrix< MeshType >::GetTriMeshData( m, F, V );
            
            igl::doublearea(V,F,dblA);            
            igl::per_face_normals(V, F, N);
            fitNormalsSinglePatch(V, N, N2);

            Eigen::MatrixXd diagDblA = dblA.array().sqrt().matrix().asDiagonal();
            double energy = (diagDblA * (N - N2)).cwiseAbs2().sum() / AreaScale();

            //std::cerr << "Gauss Image Thinness: " << energy << std::endl;

            /*
            double energy = 0;

            for(int i=0; i<N.rows(); i++){
                double dotProd = N.row(i).dot(N2.row(i));
                double angle = std::abs(std::acos(dotProd));

                if (angle / M_PI * 180. > MaxGaussImageThickness()){
                    energy += dblA(i);
                }
            }*/

            return std::max(0., energy-MaxGaussImageThickness());

        }

        if(UVMode() == PMHausdorf){
            Eigen::MatrixXd V, resultV;
            Eigen::MatrixXi F, resultF;

            vcg::tri::MeshToMatrix< MeshType >::GetTriMeshData( m, F, V );
            tri_strip::approximate_single_patch(V, F, resultV, resultF);

            double hausdorff;
            igl::hausdorff(V, F, resultV, resultF, hausdorff);

            return std::max(0., hausdorff-MaxHausdorff());
        }
    }
};

#endif
