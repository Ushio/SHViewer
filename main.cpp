#include "pr.hpp"
#include <iostream>
#include <memory>

constexpr float abs_constant( float x )
{
    return x < 0.0f ? -x : x;
}
constexpr float newton_sqrt_r( float xn, float a, int e )
{
    float xnp1 = xn - ( xn * xn - a ) * 0.5f / xn;
    float e0 = abs_constant( xn * xn - a );
    float e1 = abs_constant( xnp1 * xnp1 - a );
    return ( e1 < e0 ) 
        ? 
        newton_sqrt_r( xnp1, a, e ) 
        : 
        ( e < 4 /* magic number */ ? newton_sqrt_r( xnp1, a, e + 1 ) : xn );
}
constexpr float newton_sqrt( float x )
{
    bool valid =
        0.0f <= x &&
        x < std::numeric_limits<float>::infinity() &&
        x == x; // nan 
    return valid 
        ? 
        ( x == 0.0f ? 0.0f : newton_sqrt_r(x, x, 0) )
        : 
        std::numeric_limits<double>::quiet_NaN();
}

void sh_L4( float v[16], float x, float y, float z )
{
    constexpr float rpi = newton_sqrt( glm::pi<float>() );
    constexpr float r3 = newton_sqrt( 3.0f );
    constexpr float r15 = newton_sqrt( 15.0f );
    constexpr float r5 = newton_sqrt( 5.0f );
    constexpr float r2 = newton_sqrt( 2.0f );
    constexpr float r35 = newton_sqrt( 35.0f );
    constexpr float r105 = newton_sqrt( 105.0f );
    constexpr float r21 = newton_sqrt( 21.0f );
    constexpr float r7 = newton_sqrt( 7 );

    const float xy = x * y;
    const float yz = y * z;
    const float xz = x * z;
    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;
    const float xyz = xy * z;

    // L=0
    v[0] = + 1.0f / ( 2.0f * rpi ); // M=0

    // L=1
    v[1] = - ( r3 / ( 2.0f * rpi ) ) * y;
    v[2] = + ( r3 / ( 2.0f * rpi ) ) * z;
    v[3] = - ( r3 / ( 2.0f * rpi ) ) * x;

    // L=2
    v[4] = + ( r15 / ( 2.0f * rpi ) ) * xy;
    v[5] = - ( r15 / ( 2.0f * rpi ) ) * yz;
    v[6] = + ( r5 / ( 4.0f * rpi ) ) * ( 3.0f * z * z - 1.0f );
    v[7] = - ( r15 / ( 2.0f * rpi ) ) * xz;
    v[8] = + ( r15 / ( 4.0f * rpi ) ) * ( xx - yy );

    // L=3
    v[9]  = - ( r2 * r35 / ( 8.0f * rpi ) ) * y * ( 3.0f * xx - yy );
    v[10] = + ( r105 / ( 2.0f * rpi ) ) * xyz;
    v[11] = - ( r2 * r21 / ( 8.0f * rpi ) ) * y * ( -1.0f + 5.0f * zz );
    v[12] = + ( r7 / ( 4.0f * rpi ) ) * z * ( 5.0f * z * z - 3.0f );
    v[13] = - ( r2 * r21 / ( 8.0f * rpi ) ) * x * ( -1.0f + 5.0f * zz );
    v[14] = + ( r105 / ( 4.0f * rpi ) ) * ( xx - yy ) * z;
    v[15] = - ( r2 * r35 / ( 8.0f * rpi ) ) * x * ( xx - 3.0f * yy );
}

int main() {
    using namespace pr;

    Config config;
    config.ScreenWidth = 1920;
    config.ScreenHeight = 1080;
    config.SwapInterval = 1;
    Initialize(config);

    Camera3D camera;
    camera.origin = { 0, 0, 8 };
    camera.lookat = { 0, 0, 0 };
    camera.zUp = true;

    double e = GetElapsedTime();

    while (pr::NextFrame() == false) {
        if (IsImGuiUsingMouse() == false) {
            UpdateCameraBlenderLike(&camera);
        }

        ClearBackground(0.1f, 0.1f, 0.1f, 1);

        BeginCamera(camera);

        PushGraphicState();

        DrawGrid(GridAxis::XY, 1.0f, 10, { 128, 128, 128 });
        DrawXYZAxis(1.0f);


        float v[16];

        int N = 100;
        LinearTransform i2theta( 0.0f, N - 1, 0.0f, glm::pi<float>() );
        LinearTransform i2phi( 0.0f, N - 1, 0.0f, 2.0f * glm::pi<float>() );

        for (int l = 0; l < 4; l++)
        {
            int mN = 2 * l + 1; // number of m

            for( int m = 0; m < mN; m++ )
            {
                SetObjectTransform(glm::translate(glm::identity<glm::mat4>(), { m * 1.5f - 1.5f * l, 0, 3.0f - l * 1.5f }));
                int indices[] = { 0, 1, 4, 9 };

                for (int i = 0; i < N; i++)
                {
                    PrimBegin(PrimitiveMode::LineStrip);
                    for (int j = 0; j < N; j++)
                    {
                        float theta = i2theta( j );
                        float phi = i2phi( i );
                        float x = sin(theta) * cos(phi);
                        float y = sin(theta) * sin(phi);
                        float z = cos(theta);

                        sh_L4( v, x, y, z );

                        float value = v[ indices[l] + m ];
                        glm::vec3 p = glm::vec3(x, y, z) * std::fabs(value);

                        glm::u8vec3 color = 0.0f < value ? glm::u8vec3( 255, 0, 0 ) : glm::u8vec3( 0, 0, 255 );
                        PrimVertex( p, color);
                    }
                    PrimEnd();
                }
            }

        }

        SetObjectIdentify();

        PopGraphicState();
        EndCamera();

        BeginImGui();

        ImGui::SetNextWindowSize({ 500, 300 }, ImGuiCond_Once);
        ImGui::Begin("Panel");
        ImGui::Text("fps = %f", GetFrameRate());

        ImGui::End();

        EndImGui();
    }

    pr::CleanUp();
}
