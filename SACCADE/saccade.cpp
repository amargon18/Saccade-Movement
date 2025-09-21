#include <yarp/os/all.h> 
#include <iostream> 
#include <yarp/sig/all.h> 
#include <yarp/dev/all.h> 
#include <string> 
#include <cmath> 
#include <random>
#include <cstdlib>
#include <ctime>
#include <vector>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

class MyThread : public PeriodicThread {
    
    public:
        //CONSTRUCTOR HILO
        //CREAR PUERTO IMAGEN
        MyThread(double period): PeriodicThread(period) {
            bool ok = imagePort.open("/img");
            if(!ok){
                cerr << "Failed to open the port" << endl;
                return;
            }
        //CONECTAR AL PUERTO DEL SIMULADOR
            yarp.connect("/icubSim/cam/left", "/img");

        //CREANDO CONEXIONES
            prop.put("device", "remote_controlboard");
            prop.put("local", "/thread");
            prop.put("remote", "/icubSim/head");

        //COMENZANDO POLYDRIVER
            pd.open(prop);

        //ABRIENDO CONTROLADORES QUE UTILIZAMOS
            pd.view(ipc);
            pd.view(enc);
            pd.view(ivc);
            if (!ipc || !ivc || !enc) {
                cerr << "Error: No se pudieron inicializar los controladores" << endl;
                return;
            }
            encposition = 0.0;

            double integral_X = 0.0;
            double integral_Y = 0.0;
            double previousError_X = 0.0;
            double previousError_Y = 0.0;
            encoder_X = 0.0, encoder_Y = 0.0;
        }

    private:
        //CONSTRUCTORES
        Network yarp;
        Property prop;
        PolyDriver pd;
        IPositionControl *ipc;
        IVelocityControl *ivc;
        IEncoders *enc;
        BufferedPort<ImageOf<PixelRgb>>imagePort;

        double pos=0.0;
        double *encpos;
        double encposition;
        const double kp = 0.15, ki = 0.0, kd = 0.1;
        double integral_X, integral_Y;
        double derivative_X, derivative_Y;
        double previousError_X, previousError_Y;
        double encoder_X, encoder_Y;
        double h_encoder_X, h_encoder_Y;
        double error_X, error_Y;
        double output_X, output_Y;
        const double TOLERANCE_EYES = 5;
        const double TOLERANCE_HEAD = 10;
        

    protected:
        std::pair<double, double> controller(double error_X, double error_Y){
            const double MAX_OUTPUT = 20;

            integral_X += error_X;
            derivative_X = error_X - previousError_X;
            output_X = kp * error_X   /*ki * integral_X */ + kd * derivative_X;
            previousError_X = error_X;

            integral_Y += error_Y;
            //cout << "INTEGRAAL_Y_CONTROLLER: " << integral_Y << endl;

            derivative_Y = error_Y - previousError_Y;
            //cout << "DERIVATIVE_Y_CONTROLLER: " << derivative_Y << endl;

            output_Y = kp * error_Y + /*ki * integral_Y + */ kd * derivative_Y;
            //cout << "OUTPUT_Y_CONTROLLER: " << output_Y << endl;
            previousError_Y = error_Y;

            output_X = std::max(-MAX_OUTPUT, std::min(output_X, MAX_OUTPUT));
            output_Y = std::max(-MAX_OUTPUT, std::min(output_Y, MAX_OUTPUT));

            return std::make_pair(output_X, output_Y);
        }



        void run() override {
            //Time::delay(0.5);
                ImageOf<PixelRgb> *image = imagePort.read();

                if (image == nullptr) {
                    cerr << "Failed to read image" << endl;
                    Time::delay(0.1);  // Espera de 100ms antes de volver a intentar
                }
            

                int pixelmean_X=0;
                int pixelmean_Y=0;
                int howmanypix=0;

                 //RECORRER PIXELES IMAGEN Y BUSCA PIXELES ROJOS
                for(int x=0; x<image ->width(); x++){
                    for(int y=0; y<image ->height(); y++){
                        PixelRgb& pixel = image -> pixel(x,y);
                        if((pixel.r>2*pixel.g) && (pixel.r>2*pixel.b)){
                            pixelmean_X=pixelmean_X+x;
                            pixelmean_Y=pixelmean_Y+y;

                            howmanypix++;
                        }
                    }
                }

                if (howmanypix == 0) {
                    cerr << "No red pixels found" << endl;
                    return;
                }

                double u_X = pixelmean_X/howmanypix; //MUESTRA POS x DE LA ESFERA ROJA
                double v_Y = pixelmean_Y/howmanypix; //MUESTRA POS y DE LA ESFERA ROJA
                //cout << "Sphere position X, Y:" << u_X << " , " << v_Y << endl;

                int center_X = image->width() / 2;
                int center_Y = image->height() / 2;

                
                error_X = center_X - u_X;
                error_Y = center_Y - v_Y;
                
                
                //cout << "ERROR_X, Y:" << abs(error_X) << " , " << abs(error_Y) << endl;

                if (abs(error_X) < TOLERANCE_EYES && abs(error_Y) < TOLERANCE_EYES) {
                    //cout << "El objeto está centrado" << endl;
                    //cout << "Moviendo la cabeza..." << endl;
                    if(abs(u_X - encoder_X) < 5 && abs(v_Y - encoder_Y) < 5){
                        //cout << "Los ojos estan centrados" << endl;
                    }else{  
                        enc->getEncoder(2, &h_encoder_X);
                        enc->getEncoder(0, &h_encoder_Y);


                        
                        
                        double factor_X = std::max(0.5, std::min(1.5, error_X / 25.0));
                        double factor_Y = std::max(0.5, std::min(1.5, error_Y / 25.0));

                        double velocity_X = output_X;
                        double velocity_Y = output_Y;

                        double new_pos_h_X = (h_encoder_X + output_X)*factor_X;
                        double new_pos_h_Y = (h_encoder_Y + encoder_Y)*factor_Y;

                        ivc->velocityMove(2, velocity_X);
                        ivc->velocityMove(0, velocity_Y);

                        ipc->positionMove(2, new_pos_h_X);
                        ipc->positionMove(0, new_pos_h_Y);
    
                        enc->getEncoder(4, &encoder_X); // Encoder para el ojo en X
                        enc->getEncoder(3, &encoder_Y); // Encoder para el ojo en Y


                        double new_pos_eye_X = encoder_X - (new_pos_h_X - h_encoder_X)*0.7; // Movimiento opuesto
                        double new_pos_eye_Y = encoder_Y - (new_pos_h_Y - h_encoder_Y)*0.7;

                        //cout << "Compensando ojos: " << new_pos_eye_X << " , " << new_pos_eye_Y << endl;

                        ipc->positionMove(4, new_pos_eye_X); // Mueve ojo en X
                        ipc->positionMove(3, new_pos_eye_Y); // Mueve ojo en Y
                        }
                                        
                }else{
                    // Calcular la señal de control solo si el error es mayor que la tolerancia
                    auto result = controller(error_X, error_Y);
                    double output_X = -result.first;
                    double output_Y = result.second;

                    // Obtener posición actual de los encoders
                    enc->getEncoder(4, &encoder_X);
                    enc->getEncoder(3, &encoder_Y);

                    // Enviar comandos de movimiento
                    ipc->positionMove(4, output_X + encoder_X);
                    ipc->positionMove(3, output_Y + encoder_Y);

                    }
                    
                Time::delay(0.1);

        }
    
        void threadRelease() override {
            ipc->positionMove(4, 0);
            ipc->positionMove(3, 0);
            ipc->positionMove(2, 0);
            ipc->positionMove(0, 0);

            cout << "END OF THE THREAD" << endl;
            pd.close();
            //imagePort.close();
        }

};

// Función para generar un número aleatorio en un rango [min, max]
double randomInRange(double min, double max) { 
    return min + (rand() / (RAND_MAX / (max - min)));
}


int main(int argc, char *argv[]) { 
    srand(time(NULL));

    //INICIAR CONEXION Y CREAR CLIENTE
    Network yarp;
    RpcClient rpc;

    //ABRIR PUERTO ESFERA
    bool ok = rpc.open("/sphere");

    if(!ok){
        cerr << "Failed to open the open the sphere port" << endl;
        return 1;
    }

    yarp.connect("/sphere", "/icubSim/world");

    //BORRAR TODO LO QUE HABIA EN EL "MUNDO"
    Bottle b, reply;
    b.addString("world");
    b.addString("del");
    b.addString("all");

    // == EJECUTAR TODO LO ALMACENADO EN B EN TERMINAL
    rpc.write(b, reply);

    b.clear();
    reply.clear();

    //CREAR ESFERA
    b.addString("world");
    b.addString("mk");
    b.addString("ssph");
    b.addFloat64(0.06);
    b.addFloat64(0.2);
    b.addFloat64(0.9);
    b.addFloat64(0.8);
    b.addFloat64(1);
    b.addFloat64(0);
    b.addFloat64(0);

    cout << "Botella:" << b.toString() << endl;
    //== EJECUTAR TODO ALMACENADO EN B EN TERMINAL
    rpc.write(b, reply);

    MyThread mt(0.005);
    mt.start();

    cout<<"Parameters of the created sphere:"<< reply.toString() << endl;


    for(int i= 0; i<=6; i++){
        cout << "Iteracion:" << i << endl;
        cout << "------------------------------------------------" << endl;

        double u = randomInRange(-0.25, 0.25); // Coordenada X del píxel
        double v = randomInRange(0.6, 1.1); // Coordenada Y del píxel

        b.clear();
        b.addString("world");
        b.addString("set");
        b.addString("ssph");
        b.addInt32(1); // Identificador de la esfera
        b.addFloat64(u);
        b.addFloat64(v);
        b.addFloat64(0.8);
        rpc.write(b, reply);

        cout << "Moved sphere to :(" << u << ", " << v<< ")" << endl;
        Time::delay(7.0); //CAMBIAR A 7.0
    }
    
    mt.stop();
    return 0;
    

};

