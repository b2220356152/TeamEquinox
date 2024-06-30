import numpy as np

class KalmanFilter:
    def __init__(self, initial_state, initial_covariance, transition_matrix, observation_matrix, process_noise, measurement_noise):
        self.state_estimate = initial_state
        self.covariance_estimate = initial_covariance
        self.transition_matrix = transition_matrix
        self.observation_matrix = observation_matrix
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise

    def predict(self):
        # Predict the next state
        self.state_estimate = self.transition_matrix @ self.state_estimate
        self.covariance_estimate = self.transition_matrix @ self.covariance_estimate @ self.transition_matrix.T + self.process_noise

    def update(self, measurement):
        # Update the state estimate with a new measurement
        kalman_gain = self.covariance_estimate @ self.observation_matrix.T @ np.linalg.inv(
            self.observation_matrix @ self.covariance_estimate @ self.observation_matrix.T + self.measurement_noise)
        self.state_estimate = self.state_estimate + kalman_gain @ (measurement - self.observation_matrix @ self.state_estimate)
        self.covariance_estimate = (np.eye(len(self.covariance_estimate)) - kalman_gain @ self.observation_matrix) @ self.covariance_estimate


class Cezeri(CezeriParent):

    start_point = []
    route = []
    turning_point = []
    modes = []
    nav_data = []
    ind = []
    landing = []
    dest_x = []
    dest_y =[]
    wait_counter = []
    set_trigger = 1
    annoying_bug = 'there'



    def __init__(self, id = 0):
        super().__init__(id = id, keyboard = False, sensor_mode = NORMAL)
        self.kalman_filter_gnss = KalmanFilter(
            initial_state=np.array([self.gnss.enlem, self.gnss.boylam, self.gnss.irtifa, 0, 0, 0]),  # [latitude, longitude, altitude, velocity_lat, velocity_lon, velocity_alt]
            initial_covariance=np.eye(6),
            transition_matrix=np.array([[1, 0, 0, 1, 0, 0],
                                        [0, 1, 0, 0, 1, 0],
                                        [0, 0, 1, 0, 0, 1],
                                        [0, 0, 0, 1, 0, 0],
                                        [0, 0, 0, 0, 1, 0],
                                        [0, 0, 0, 0, 0, 1]]),
            observation_matrix=np.array([[1, 0, 0, 0, 0, 0],
                                         [0, 1, 0, 0, 0, 0],
                                         [0, 0, 1, 0, 0, 0]]),
            process_noise=np.eye(6) * 0.1,
            measurement_noise=np.eye(3) * 0.5
        )

        self.kalman_filter_magnetometer = KalmanFilter(
            initial_state=np.array([self.manyetometre.veri, 0]),  # [magnetometer reading, change_rate]
            initial_covariance=np.eye(2),
            transition_matrix=np.array([[1, 1],
                                        [0, 1]]),
            observation_matrix=np.array([[1, 0]]),
            process_noise=np.eye(2) * 0.1,
            measurement_noise=np.eye(1) * 0.5
        )

    def get_noisy_measurement_gnss(self):
        # Simulate getting a noisy GNSS measurement
        noise = np.random.normal(0, 0.5, 3)  # Assuming a noise standard deviation of 0.5
        return np.array([self.gnss.enlem, self.gnss.boylam, self.gnss.irtifa]) + noise

    def get_noisy_measurement_magnetometer(self):
        # Simulate getting a noisy magnetometer measurement
        noise = np.random.normal(0, 0.5, 1)  # Assuming a noise standard deviation of 0.5
        return np.array([self.manyetometre.veri]) + noise
    
    def set_cezeri(self):
        i = self.id-1
        self.start_point.append(self.harita.bolge(self.gnss.enlem,self.gnss.boylam))
        self.dest_x.append(self.hedefler[0].bolge.enlem) 
        self.dest_y.append(self.hedefler[0].bolge.boylam)
        self.route.append(self._get_route()) 
        self.turning_point.append(self._get_turning_pionts(self.route[i])) 
        self.modes.append(self._get_nav_details(self.turning_point[i])[0])
        self.nav_data.append(self._get_nav_details(self.turning_point[i])[1])
        self.ind.append(0)
        self.landing.append(0)
        self.wait_counter.append(0)
        print(self.start_point[i].enlem,self.start_point[i].boylam,'ID: ',self.id,i,len(self.route))
        print(self.turning_point[i],self.id)
        #print(self.hedefler[0].bolge.enlem,self.hedefler[0].bolge.boylam,'ID: ',self.id,i)

    def reset_destination(self):
        self.route[self.id] = self._get_route()
        self.turning_point[self.id] = self._get_turning_pionts(self.route[self.id])
        self.modes[self.id] = self._get_nav_details(self.turning_point[self.id])[0]
        self.nav_data[self.id] = self._get_nav_details(self.turning_point[self.id])[1]
        self.ind[self.id] = 0
        self.landing[self.id] = 0
        self.dest_x[self.id] = self.hedefler[0].bolge.enlem
        self.dest_y[self.id] = self.hedefler[0].bolge.boylam
        self.wait_counter[self.id] = 0

    def _get_route(self):
        
        p=0
        q=0
        distances_from_start = np.ones((100,100)) * np.inf
        obstacle_map = np.zeros((100,100))
        parent = np.zeros((100,100,2))
        route =[[]]
        for i in range(990,-1010,-20):
            q=0
            for j in range(-990,1010,20):
                bolge = self.harita.bolge(i,j)
                if bolge.ucusa_yasakli_bolge or bolge.ruzgar or (bolge.yukselti>119):
                    obstacle_map[p][q] = 1
                q = q+1
            p = p+1
       
        start_pnt_x = int(49.5-self.start_point[self.id-1].enlem/20)
        start_pnt_y = int(49.5+self.start_point[self.id-1].boylam/20)
        dest_pnt_x = int(49.5 -  self.dest_x[self.id-1]/20)
        dest_pnt_y = int(49.5 + self.dest_y[self.id-1]/20)
        distances_from_start[start_pnt_x][start_pnt_y] = 0
        obstacle_map[start_pnt_x][start_pnt_y] = 5
        obstacle_map[dest_pnt_x][dest_pnt_y] = 6
        

        

        while True:
            obstacle_map[start_pnt_x][start_pnt_y] = 5
            obstacle_map[dest_pnt_x][dest_pnt_y] = 6
            min_dist = np.min(distances_from_start)
            current = np.unravel_index(np.argmin(distances_from_start, axis=None), distances_from_start.shape)
            if ((current[0] == dest_pnt_x) and (current[1]== dest_pnt_y)) or np.isinf(min_dist):
                break
            obstacle_map[current] = 2
            distances_from_start[current] = np.inf
            A=[-1,1,0,0];
            B=[0,0,-1,1];
            

            for i in range(4):
                if (current[0]+A[i]>=0) and (current[0]+A[i]<100) and (current[1]+B[i]>=0) and (current[1]+B[i]<100):
                    if (obstacle_map[current[0]+A[i]][current[1]+B[i]] == 0) :
                        if i == 1:
                            distances_from_start[current[0]+A[i]][current[1]+B[i]] = min_dist+1
                        elif i == 0:
                            distances_from_start[current[0]+A[i]][current[1]+B[i]] = min_dist+1
                        else:
                            distances_from_start[current[0]+A[i]][current[1]+B[i]] = min_dist+1

                        parent[current[0]+A[i]][current[1]+B[i]] = current
                        obstacle_map[current[0]+A[i]][current[1]+B[i]] = 3
                        
                    elif obstacle_map[current[0]+A[i]][current[1]+B[i]] == 6:
                        distances_from_start[dest_pnt_x][dest_pnt_y] = 0
                        parent[current[0]+A[i]][current[1]+B[i]] = current
                        
        
        
        
        if np.isinf(distances_from_start[dest_pnt_x][dest_pnt_y]) == 0:
            
            route[0] = [dest_pnt_x,dest_pnt_y]
            
            while (parent[int(route[0][0])][int(route[0][1])][0] != 0) and (parent[int(route[0][0])][int(route[0][1])][1] != 0):
                route = np.append([parent[int(route[0][0])][int(route[0][1])]],route, axis=0)
        for i in range(len(route)):
            route[i][0] = 990-(route[i][0]*20)
            route[i][1] = -990+(route[i][1]*20)       
        return route

    def _get_turning_pionts(self,route):
        points=[[]]
        points[0] = route[0]
        current_point = route[0]

        for i in range(1,len(route)):
            if (abs(current_point[0]-route[i][0]) > 10) and (abs(current_point[1]-route[i][1]) > 10):
                points = np.append(points,[route[i-1]],axis=0)
                current_point = route[i-1]
        points = np.append(points,[route[len(route)-1]],axis=0)
        return points

    def _get_nav_details(self,turning_point):
        angle = 0
        modes = [[0 for _ in range(2)] for _ in range(len(turning_point)-1)]
        nav_data = [[0 for _ in range(4)] for _ in range(len(turning_point)-1)]
        for i in range(1,len(turning_point)):
            nav_data[i-1][0] = turning_point[i][0]
            nav_data[i-1][1] = turning_point[i][1]

            

            if angle == 0:
                if turning_point[i][0]-turning_point[i-1][0]>0:
                    modes[i-1][1] = 'straight'
                    nav_data[i-1][3] = 0
                elif turning_point[i][0]-turning_point[i-1][0]<0:
                    modes[i-1][1] = 'back_turn'
                    nav_data[i-1][3] = 3.14
                elif turning_point[i][1]-turning_point[i-1][1]>0:
                    modes[i-1][1] = 'right'
                    nav_data[i-1][3] = 1.57
                elif turning_point[i][1]-turning_point[i-1][1]<0:
                    modes[i-1][1] = 'left'
                    nav_data[i-1][3] = -1.57


            elif angle == 1.57:
                if turning_point[i][0]-turning_point[i-1][0]>0:
                    modes[i-1][1] = 'left'
                    nav_data[i-1][3] = 0
                elif turning_point[i][0]-turning_point[i-1][0]<0:
                    modes[i-1][1] = 'right'
                    nav_data[i-1][3] = 3.14
                elif turning_point[i][1]-turning_point[i-1][1]>0:
                    modes[i-1][1] = 'straight'
                    nav_data[i-1][3] = 1.57
                elif turning_point[i][1]-turning_point[i-1][1]<0:
                    modes[i-1][1] = 'back_turn'
                    nav_data[i-1][3] = -1.57


            elif angle == 3.14:
                if turning_point[i][0]-turning_point[i-1][0]>0:
                    modes[i-1][1] = 'back_turn'
                    nav_data[i-1][3] = 0
                elif turning_point[i][0]-turning_point[i-1][0]<0:
                    modes[i-1][1] = 'straight'
                    nav_data[i-1][3] = 3.14
                elif turning_point[i][1]-turning_point[i-1][1]>0:
                    modes[i-1][1] = 'left'
                    nav_data[i-1][3] = 1.57
                elif turning_point[i][1]-turning_point[i-1][1]<0:
                    modes[i-1][1] = 'right'
                    nav_data[i-1][3] = -1.57 

            elif angle == -1.57:
                if turning_point[i][0]-turning_point[i-1][0]>0:
                    modes[i-1][1] = 'right'
                    nav_data[i-1][3] = 0
                elif turning_point[i][0]-turning_point[i-1][0]<0:
                    modes[i-1][1] = 'left'
                    nav_data[i-1][3] = 3.14
                elif turning_point[i][1]-turning_point[i-1][1]>0:
                    modes[i-1][1] = 'back_turn'
                    nav_data[i-1][3] = 1.57
                elif turning_point[i][1]-turning_point[i-1][1]<0:
                    modes[i-1][1] = 'straight'
                    nav_data[i-1][3] = -1.57

            
            if 9<abs(turning_point[i][0]-turning_point[i-1][0]) + abs(turning_point[i][1]-turning_point[i-1][1]) < 100:
                modes[i-1][0] = 'slide'
                nav_data[i-1][2] = 5.5
                if (modes[i-1][1] == 'right') or (modes[i-1][1] == 'left'):
                    nav_data[i-1][3] = angle
            else:
                modes[i-1][0] = 'normal'
                nav_data[i-1][2] = 6

            angle = nav_data[i-1][3]
        return modes,nav_data
          
    def cezeri_go(self):

         # Obtain a noisy GNSS measurement
        measurement_gnss = self.get_noisy_measurement_gnss()
        # Obtain a noisy magnetometer measurement
        measurement_magnetometer = self.get_noisy_measurement_magnetometer()

        # Predict the next state
        self.kalman_filter_gnss.predict()
        self.kalman_filter_magnetometer.predict()

        # Update the state with the noisy measurements
        self.kalman_filter_gnss.update(measurement_gnss)
        self.kalman_filter_magnetometer.update(measurement_magnetometer)

        # Use the filtered state for navigation
        filtered_state_gnss = self.kalman_filter_gnss.state_estimate
        filtered_state_magnetometer = self.kalman_filter_magnetometer.state_estimate

        self.gnss.enlem, self.gnss.boylam, self.gnss.irtifa = filtered_state_gnss[0], filtered_state_gnss[1], filtered_state_gnss[2]
        self.manyetometre.veri = filtered_state_magnetometer[0]
        if (self.set_trigger == 1):
            self.set_cezeri()
            self.set_trigger = 0

        elif ((self.dest_x[self.id-1] != self.hedefler[0].bolge.enlem) or (self.dest_y[self.id-1] != self.hedefler[0].bolge.boylam)) and (self.annoying_bug == 'gone'):
            self.reset_destination()

        elif ((self.yerel_harita[1].trafik) or (self.yerel_harita[3].trafik) or (self.yerel_harita[5].trafik)) and (self.wait_counter[self.id-1] <= (self.id*100000)):
            self.dur()
            self.wait_counter[self.id-1] = self.wait_counter[self.id-1] + 1
            print('waiting')

        elif (self.gnss.irtifa <113) and (self.landing[self.id-1] == 0):
            self.wait_counter[self.id-1] = 0
            self.yukari_git(HIZLI)

        elif self.ind[self.id-1] == len(self.nav_data[self.id-1]):
            self.wait_counter[self.id-1] = 0
            self.dur()
            self.landing[self.id-1] = 1
            if self.lidar.mesafe > 1.1:
                self.asagi_git(YAVAS)

        elif self.modes[self.id-1][self.ind[self.id-1]][1] == 'back_turn':
            self.wait_counter[self.id-1] = 0
            self.dur()
            if abs(self.manyetometre.veri - self.nav_data[self.id-1][self.ind[self.id-1]][3]) > 0.035:
                self.dur()
                self.don(3.14/9.42)
            else:
                self.dur()
                if (abs(self.gnss.enlem-self.nav_data[self.id-1][self.ind[self.id-1]][0]) >self.nav_data[self.id-1][self.ind[self.id-1]][2]) or (abs(self.gnss.boylam-self.nav_data[self.id-1][self.ind[self.id-1]][1]) >self.nav_data[self.id-1][self.ind[self.id-1]][2]):
                    self.ileri_git(HIZLI)
                else:
                    self.dur()
                    self.ind[self.id-1] = self.ind[self.id-1] + 1



        elif self.modes[self.id-1][self.ind[self.id-1]][0] == 'slide':
            
            self.wait_counter[self.id-1] = 0
            self.dur()
            if self.modes[self.id-1][self.ind[self.id-1]][1] == 'left':
                self.dur()
                if (abs(self.gnss.enlem-self.nav_data[self.id-1][self.ind[self.id-1]][0]) >self.nav_data[self.id-1][self.ind[self.id-1]][2]) or (abs(self.gnss.boylam-self.nav_data[self.id-1][self.ind[self.id-1]][1]) >self.nav_data[self.id-1][self.ind[self.id-1]][2]):
                    self.sola_git(HIZLI)
                else:
                    self.dur()
                    self.ind[self.id-1] = self.ind[self.id-1] + 1
            elif self.modes[self.id-1][self.ind[self.id-1]][1] == 'right':
                self.dur()
                if (abs(self.gnss.enlem-self.nav_data[self.id-1][self.ind[self.id-1]][0]) >self.nav_data[self.id-1][self.ind[self.id-1]][2]) or (abs(self.gnss.boylam-self.nav_data[self.id-1][self.ind[self.id-1]][1]) >self.nav_data[self.id-1][self.ind[self.id-1]][2]):
                    self.saga_git(HIZLI)
                else:
                    self.dur()
                    self.ind[self.id-1] = self.ind[self.id-1] + 1
            elif self.modes[self.id-1][self.ind[self.id-1]][1] == 'straight':
                
                self.dur()
                if (abs(self.gnss.enlem-self.nav_data[self.id-1][self.ind[self.id-1]][0]) >self.nav_data[self.id-1][self.ind[self.id-1]][2]) or (abs(self.gnss.boylam-self.nav_data[self.id-1][self.ind[self.id-1]][1]) >self.nav_data[self.id-1][self.ind[self.id-1]][2]):
                    self.ileri_git(HIZLI)
                else:
                    self.dur()
                    self.ind[self.id-1] = self.ind[self.id-1] + 1
        
        elif self.modes[self.id-1][self.ind[self.id-1]][0] == 'normal':
            self.wait_counter[self.id-1] = 0
            self.dur()
            if self.modes[self.id-1][self.ind[self.id-1]][1] == 'left':
                if (abs(abs(self.manyetometre.veri)-abs(self.nav_data[self.id-1][self.ind[self.id-1]][3])) > 0.035):
                    self.don(-3.14/9.42)
                else: 
                    self.dur()
                    self.modes[self.id-1][self.ind[self.id-1]][1] = 'straight'
            elif self.modes[self.id-1][self.ind[self.id-1]][1] == 'right':
                if (abs(abs(self.manyetometre.veri)-abs(self.nav_data[self.id-1][self.ind[self.id-1]][3])) > 0.035):
                    self.don(3.14/9.42)
                else: 
                    self.dur()
                    self.modes[self.id-1][self.ind[self.id-1]][1] = 'straight'
            elif self.modes[self.id-1][self.ind[self.id-1]][1] == 'straight':
                if (abs(self.gnss.enlem-self.nav_data[self.id-1][self.ind[self.id-1]][0]) >self.nav_data[self.id-1][self.ind[self.id-1]][2]) or (abs(self.gnss.boylam-self.nav_data[self.id-1][self.ind[self.id-1]][1]) >self.nav_data[self.id-1][self.ind[self.id-1]][2]):
                    self.ileri_git(HIZLI)
                else:
                    self.dur()
                    self.ind[self.id-1] = self.ind[self.id-1] + 1
        self.annoying_bug = 'gone'
          
    def test(self):
        
        if self.gnss.irtifa<90:
            self.yukari_git(HIZLI)
        else:
            self.dur()
        #print(self.imu.hiz)
        #print(self.yerel_harita[1])
        print(self.gnss.enlem, self.gnss.boylam,self.gnss.irtifa,'ID: ',self.id)
        self.annoying_bug = 'gone'
    
    def run(self):
        print(self.manyetometre)
        print(self.gnss.enlem, self.gnss.boylam,self.gnss.irtifa)
        #print(self.start_point.enlem,self.start_point.boylam)
        #print(self.hedefler[0])
        #print(self.hedefler[0].bolge.enlem,self.hedefler[0].bolge.boylam,self.id)
        #self.current_position = self.harita.bolge(self.gnss.enlem, self.gnss.boylam)
        #print('current_pos_yukselti=',self.current_position.yukselti)
        #bolge = self.harita.bolge(990,-990)
        #print(bolge)
        self.cezeri_go()
        #print(self.yerel_harita[1])
        #self.test()
        #print(self.turning_point,self.id)
        #print(self.nav_data)

cezeri_1 = Cezeri(id = 1)

while robot.is_ok():
    (cezeri_1.run())
    robot.is_ok()
        
    
    
    
   
