#include "object_tracking.h"

Category_Of_Points::Category_Of_Points(sensor_msgs::PointCloud point_cloud,int name)
:last_name_(-1)
{
    point_cloud_.header.frame_id = point_cloud.header.frame_id;
    point_cloud_.points.resize(point_cloud.points.size());
    centrol_x_ = 0.0;
    centrol_y_ = 0.0;
    centrol_z_ = 0.0;
    for(int i = 0;i < point_cloud.points.size();i++)
    {
        point_cloud_.points[i].x = point_cloud.points[i].x;
        point_cloud_.points[i].y = point_cloud.points[i].y;
        point_cloud_.points[i].z = point_cloud.points[i].z;

        centrol_x_ += (point_cloud.points[i].x / point_cloud.points.size());
        centrol_y_ += (point_cloud.points[i].y / point_cloud.points.size());
        centrol_z_ += (point_cloud.points[i].z / point_cloud.points.size());
    }
    name_ = name;
    heredity_name_ - name;
    tx_ = ros::Time::now().toSec();
}

Category_Of_Points::~Category_Of_Points()
{}

void Category_Of_Points::setName(int name)
{
    name_ = name;
}

void Category_Of_Points::setVelocity(geometry_msgs::Point velocity)
{
    linear_velocity_.x = velocity.x;
    linear_velocity_.y = velocity.y;
    linear_velocity_.z = velocity.z;
}

///////////////////////////////////////////////----------------------------////////////////////////////////////////

Object_Tracking::Object_Tracking()
:cluster_threshold_slope_(0.15),w1_(1),w2_(1),var_of_points_bias_(25),var_of_distance_bias_(0.01)
,predicted_variance_(1),measure_variance_(1)
{
    dynamic_reconfigure::Server<point_cloud_test::point_cloud_processe_Config> server;
    dynamic_reconfigure::Server<point_cloud_test::point_cloud_processe_Config>::CallbackType f;
    f = boost::bind(&Object_Tracking::dynamicCb,this,_1,_2);
    server.setCallback(f);
    sub_points_ = n_.subscribe("/object_point_cloud",1,&Object_Tracking::subPointsCb,this);
    pub_vel_ = n_.advertise<geometry_msgs::PoseArray>("/object_velocity",1);
    pub_rviz_box_ = n_.advertise<visualization_msgs::MarkerArray>("/point_category",1);
    pub_rqt_plot_ = n_.advertise<std_msgs::Float32>("rqt_plot_test",1);
    pub_points_state_ = n_.advertise<geometry_msgs::PoseArray>("/points_state",1);

    ros::spin();
}

Object_Tracking::~Object_Tracking()
{}

void Object_Tracking::pointCloudClassification(sensor_msgs::PointCloud point_cloud)
{
    if(point_cloud_.size() == 0)
    {
        return;
    }
    //cout << "begin to classificate the point cloud." << endl;
    if(category_of_points_of_time_.size() >= 10)
    {
        for(int i = 0;i < category_of_points_of_time_[0].size();i++)
        {
            delete category_of_points_of_time_[0][i];
        }
        category_of_points_of_time_.erase(category_of_points_of_time_.begin(),category_of_points_of_time_.begin() + 1);
    }

    vector<Category_Of_Points*> new_category;
    while((point_cloud_.size() != 0) && ros::ok())
    {
        sensor_msgs::PointCloud templant_points;
        templant_points.header.frame_id = point_cloud.header.frame_id;
        for(int i = 0;i < point_cloud_.size();i++)
        {
            if(templant_points.points.size() == 0)
            {
                geometry_msgs::Point32 point1;
                point1.x = point_cloud_[i].x;
                point1.y = point_cloud_[i].y;
                point1.z = point_cloud_[i].z;
                templant_points.points.push_back(point1);
                point_cloud_.erase(point_cloud_.begin() + i,point_cloud_.begin() + i + 1);
                i--;
                continue;
            }

            if(minDistanceOfPointToCloud(point_cloud_[i],templant_points) < cluster_threshold_slope_ * distancePointToLidar(point_cloud_[i]))
            {
                geometry_msgs::Point32 point1;
                point1.x = point_cloud_[i].x;
                point1.y = point_cloud_[i].y;
                point1.z = point_cloud_[i].z;
                templant_points.points.push_back(point1);
                point_cloud_.erase(point_cloud_.begin() + i,point_cloud_.begin() + i + 1);
                i--;
            }
        }

        new_category.push_back(new Category_Of_Points(templant_points,new_category.size()));
    }
    category_of_points_of_time_.push_back(new_category);

    // if(category_of_points_of_time_.size() > 5)
    // {
    // cout << "display the time:" << category_of_points_of_time_[category_of_points_of_time_.size() - 1][0]->tx_ -
    //                                category_of_points_of_time_[category_of_points_of_time_.size() - 2][0]->tx_ << endl;
    // }

    correlationOperate();

    pubPointsState();

    displayInRviz();
}

double Object_Tracking::minDistanceOfPointToCloud(geometry_msgs::Point32 point,sensor_msgs::PointCloud cloud)
{
    double ans = 100000;
    int n;
    for(int i = 0;i < cloud.points.size();i++)
    {
        double distance = sqrt((point.x - cloud.points[i].x) * (point.x - cloud.points[i].x) 
                             + (point.y - cloud.points[i].y) * (point.y - cloud.points[i].y) 
                             + (point.z - cloud.points[i].z) * (point.z - cloud.points[i].z));
        if(ans > distance)
        {
            ans = distance;
            n = i;
        }
    }

    // cout << cloud.points[n].x << "," << cloud.points[n].y << "," << cloud.points[n].z << endl
    //      << point.x << "," << point.y << "," << point.z << endl
    //      << ans << endl << endl;
    return ans;
}

double Object_Tracking::distancePointToLidar(geometry_msgs::Point32 point)
{
    double ans = 0;
    ans = sqrt((point.x - point_lidar_.x) * (point.x - point_lidar_.x) 
             + (point.y - point_lidar_.y) * (point.y - point_lidar_.y) 
             + (point.z - point_lidar_.z) * (point.z - point_lidar_.z));
    
    // cout << point_lidar_.x << "," << point_lidar_.y << "," << point_lidar_.z << endl
    //      << point.x << "," << point.y << "," << point.z << endl
    //      << ans << endl;
    return ans;
}

void Object_Tracking::pubPointsState()
{
    geometry_msgs::PoseArray poses;
    poses.header.frame_id = category_of_points_of_time_[0][0]->point_cloud_.header.frame_id;
    geometry_msgs::Pose pose1;
    pose1.position.x = point_lidar_.x;
    pose1.position.y = point_lidar_.y;
    pose1.position.z = point_lidar_.z;
    poses.poses.push_back(pose1);

    int n = category_of_points_of_time_.size() - 1;
    for(int i = 0;i < category_of_points_of_time_[n].size();i++)
    {
        for(int j = 0;j < category_of_points_of_time_[n][i]->point_cloud_.points.size();j++)
        {
            pose1.position.x = category_of_points_of_time_[n][i]->point_cloud_.points[j].x;
            pose1.position.y = category_of_points_of_time_[n][i]->point_cloud_.points[j].y;
            pose1.position.z = category_of_points_of_time_[n][i]->point_cloud_.points[j].z;
            pose1.orientation.x = category_of_points_of_time_[n][i]->linear_velocity_.x;
            pose1.orientation.y = category_of_points_of_time_[n][i]->linear_velocity_.y;
            poses.poses.push_back(pose1);
        }
    }

    pub_points_state_.publish(poses);
}

void Object_Tracking::displayInRviz()
{
    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = category_of_points_of_time_[0][0]->point_cloud_.header.frame_id;
    int n_ = category_of_points_of_time_.size() - 1;
    for(int i = 0;i < category_of_points_of_time_[n_].size();i++)
    {
        geometry_msgs::Pose pose1;
        pose1.position.x = category_of_points_of_time_[n_][i]->centrol_x_;
        pose1.position.y = category_of_points_of_time_[n_][i]->centrol_y_;
        double cos_theta = category_of_points_of_time_[n_][i]->linear_velocity_.x / sqrt(category_of_points_of_time_[n_][i]->linear_velocity_.x * category_of_points_of_time_[n_][i]->linear_velocity_.x 
                                                                                       + category_of_points_of_time_[n_][i]->linear_velocity_.y * category_of_points_of_time_[n_][i]->linear_velocity_.y);
        if(category_of_points_of_time_[n_][i]->linear_velocity_.y < 0) pose1.orientation.w = 0 - sqrt((1 + cos_theta) / 2);
        else pose1.orientation.w = sqrt((1 + cos_theta) / 2);
        pose1.orientation.z = sqrt(1 - pose1.orientation.w * pose1.orientation.w);
        pose_array.poses.push_back(pose1);
    }
    pub_vel_.publish(pose_array);

    ///////////////////////////////////////////////////////////////
    double vvv = category_of_points_of_time_[0][0]->linear_velocity_.x * category_of_points_of_time_[0][0]->linear_velocity_.x
               + category_of_points_of_time_[0][0]->linear_velocity_.y * category_of_points_of_time_[0][0]->linear_velocity_.y;
    rqtPlotDisplay(sqrt(vvv));
    ///////////////////////////////////////////////////////////////

    visualization_msgs::MarkerArray boxs;
    visualization_msgs::Marker box1;
    box1.header.frame_id = category_of_points_of_time_[0][0]->point_cloud_.header.frame_id;
    int n = category_of_points_of_time_.size() - 1;
    if(n == -1)
    {
        cout << "no data." << endl;
        return;
    }
    for(int i = 0;i < category_of_points_of_time_[n].size();i++)
    {
        double centrol_x = 0;
        double centrol_y = 0;
        double min_x = 10000,min_y = 10000;
        double max_x = -10000,max_y = -10000;
        for(int j = 0;j < category_of_points_of_time_[n][i]->point_cloud_.points.size();j++)
        {
            centrol_x += category_of_points_of_time_[n][i]->point_cloud_.points[j].x;
            centrol_y += category_of_points_of_time_[n][i]->point_cloud_.points[j].y;
            min_x = min_x > category_of_points_of_time_[n][i]->point_cloud_.points[j].x ? category_of_points_of_time_[n][i]->point_cloud_.points[j].x : min_x;
            min_y = min_y > category_of_points_of_time_[n][i]->point_cloud_.points[j].y ? category_of_points_of_time_[n][i]->point_cloud_.points[j].y : min_y;
            max_x = max_x < category_of_points_of_time_[n][i]->point_cloud_.points[j].x ? category_of_points_of_time_[n][i]->point_cloud_.points[j].x : max_x;
            max_y = max_y < category_of_points_of_time_[n][i]->point_cloud_.points[j].y ? category_of_points_of_time_[n][i]->point_cloud_.points[j].y : max_y;
        }
        centrol_x /= category_of_points_of_time_[n][i]->point_cloud_.points.size();
        centrol_y /= category_of_points_of_time_[n][i]->point_cloud_.points.size();
        double width = max_x - min_x;
        double height = max_y - min_y;
        if(width < 0.1) width = 0.1;
        if(height < 0.1) height = 0.1;

        box1.id = boxs.markers.size();
        box1.type = visualization_msgs::Marker::CUBE;
        box1.action = visualization_msgs::Marker::ADD;
        box1.pose.position.x = centrol_x;
        box1.pose.position.y = centrol_y;
        box1.pose.position.z = 0;
        box1.pose.orientation.x = 0.0;
        box1.pose.orientation.y = 0.0;
        box1.pose.orientation.z = 0.0;
        box1.pose.orientation.w = 1.0;
        box1.scale.x = width;
        box1.scale.y = height;
        box1.scale.z = 0.1;
        box1.color.r = 0.0f;
        box1.color.g = 1.0f;
        box1.color.b = 0.0f;
        box1.color.a = 0.5;
        boxs.markers.push_back(box1);

        box1.id = boxs.markers.size();
        box1.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        box1.action = visualization_msgs::Marker::ADD;
        box1.pose.position.x = centrol_x;
        box1.pose.position.y = centrol_y;
        box1.pose.position.z = 0.1;
        box1.pose.orientation.x = 0.0;
        box1.pose.orientation.y = 0.0;
        box1.pose.orientation.z = 0.0;
        box1.pose.orientation.w = 1.0;
        box1.scale.x = 0.2;
        box1.scale.y = 0.2;
        box1.scale.z = 0.2;
        box1.color.r = 1.0f;
        box1.color.g = 0.0f;
        box1.color.b = 0.0f;
        box1.color.a = 1.0;
        stringstream ss;
        ss << category_of_points_of_time_[n][i]->last_name_ << "->" << category_of_points_of_time_[n][i]->name_ << "(" 
            << sqrt(category_of_points_of_time_[n][i]->linear_velocity_.x * category_of_points_of_time_[n][i]->linear_velocity_.x 
                  + category_of_points_of_time_[n][i]->linear_velocity_.y * category_of_points_of_time_[n][i]->linear_velocity_.y) << ")";
        ss >> box1.text;
        boxs.markers.push_back(box1);
    }
    
    for(int i = 0;i < 10;i++)
    {
        box1.id = boxs.markers.size();
        box1.type = visualization_msgs::Marker::CUBE;
        box1.action = visualization_msgs::Marker::ADD;
        box1.pose.position.x = 0;
        box1.pose.position.y = 0;
        box1.pose.position.z = 0;
        box1.pose.orientation.x = 0.0;
        box1.pose.orientation.y = 0.0;
        box1.pose.orientation.z = 0.0;
        box1.pose.orientation.w = 1.0;
        box1.scale.x = 0.1;
        box1.scale.y = 0.1;
        box1.scale.z = 0.1;
        box1.color.r = 0.0f;
        box1.color.g = 1.0f;
        box1.color.b = 0.0f;
        box1.color.a = 0.0;
        boxs.markers.push_back(box1);
    }
    pub_rviz_box_.publish(boxs);
    
    //cout << "the number of category is:" << category_of_points_of_time_[n].size() << endl;
}

void Object_Tracking::correlationOperate()
{
    int t = category_of_points_of_time_.size() - 1;
    if(t < 5) return;

    for(int i = 0;i < correlation_of_category_.size();i++)
    {
        correlation_of_category_[i].clear();
    }
    correlation_of_category_.clear();
    correlation_of_category_.resize(category_of_points_of_time_[t].size());
    for(int i = 0;i < correlation_of_category_.size();i++)
    {
        correlation_of_category_[i].resize(category_of_points_of_time_[t - 1].size() + 1);
    }

    for(int i = 0;i < correlation_of_category_.size();i++)  //calculate the correlation matrix
    {
        for(int j = 0;j < correlation_of_category_[i].size();j++)
        {
            correlation_of_category_[i][j] = correlationOfFun(i,j,t);
        }
    }

    for(int i = 0;i < correlation_of_category_.size();i++)  //print the correlation matrix
    {
        for(int j = 0;j < correlation_of_category_[i].size();j++)
        {
            cout << correlation_of_category_[i][j] << "  ";
        }
        cout << endl;
    }
    cout << endl;

    connectWithLastName();  //connect the correlation category's name

    estimateTheVelocity();  //connect the correlation category's velocity
}

double Object_Tracking::correlationOfFun(int now,int last,int period)
{
    double ans = 0;
    if(last == correlation_of_category_[now].size() - 1)
    {
        double max_bel = 0;
        for(int i = 0;i < correlation_of_category_[now].size() - 1;i++)
        {
            max_bel = max_bel < correlation_of_category_[now][i] ? correlation_of_category_[now][i]:max_bel;
        }
        ans = 0.7 * (w1_ + w2_ - max_bel); /////////////////////////////////////////////////////////////////////////////
    }
    else
    {
        ans = w1_ * fun1(now,last,period) + w2_ * fun2(now,last,period);///////////////////////////////////////////
    }
    return ans;
}

double Object_Tracking::fun1(int now,int last,int period)
{
    double ans = 0;
    int points_bias = category_of_points_of_time_[period][now]->point_cloud_.points.size() - category_of_points_of_time_[period - 1][last]->point_cloud_.points.size();
    //ans = 1 / sqrt(var_of_points_bias_) / sqrt(2 * 3.14159265) * exp(0 - points_bias * points_bias / 2 / var_of_points_bias_);
    ans = exp(0 - points_bias * points_bias / var_of_points_bias_);
    return ans;
}

double Object_Tracking::fun2(int now,int last,int period)
{
    double ans = 0;
    double det_time = category_of_points_of_time_[period][0]->tx_ - category_of_points_of_time_[period - 1][0]->tx_;
    det_time = det_time < 0.00001 ? 0.1 : det_time;
    double distance;
    double det_x;
    double det_y;
    det_x = category_of_points_of_time_[period][now]->centrol_x_ - category_of_points_of_time_[period - 1][last]->centrol_x_ - category_of_points_of_time_[period - 1][last]->linear_velocity_.x * det_time;
    det_y = category_of_points_of_time_[period][now]->centrol_y_ - category_of_points_of_time_[period - 1][last]->centrol_y_ - category_of_points_of_time_[period - 1][last]->linear_velocity_.y * det_time;
    distance = sqrt(det_x * det_x + det_y * det_y);
    //ans = 1 / sqrt(var_of_distance_bias_) / sqrt(2 * 3.14159265) * exp(0 - distance * distance / 2 / var_of_distance_bias_);
    ans = exp(0 - distance * distance / var_of_distance_bias_);

    return ans;
}

int Object_Tracking::nameIsUsed(int name,int period)
{
    for(int i = 0;i < category_of_points_of_time_[period - 1].size();i++)
    {
        if(name == category_of_points_of_time_[period - 1][i]->heredity_name_)
        {
            return 1;
        }
    }
    for(int i = 0;i < category_of_points_of_time_[period].size();i++)
    {
        if(name == category_of_points_of_time_[period][i]->heredity_name_)
        {
            return 1;
        }
    }

    return 0;
}

void Object_Tracking::connectWithLastName()
{
    int t = category_of_points_of_time_.size() - 1;
    if(t < 5) return;

    for(int i = 0;i < correlation_of_category_.size();i++)     //connect the correlation category's name
    {
        int similatest_category = -2;
        double max_bel = 0;
        for(int j = 0;j < correlation_of_category_[i].size();j++)
        {
            if(max_bel < correlation_of_category_[i][j])
            {
                max_bel = correlation_of_category_[i][j];
                similatest_category = j;
            }
        }

        if(similatest_category == correlation_of_category_[i].size() - 1)
        {
            category_of_points_of_time_[t][i]->last_name_ = -1;
            int new_name = 0;
            while(nameIsUsed(new_name,t))
            {
                new_name++;
            }
            category_of_points_of_time_[t][i]->heredity_name_ = new_name;
        }
        else
        {
            if(similatest_category < 0)
            {
                cout << "error!!!" << endl;
                return;
            }
            category_of_points_of_time_[t][i]->last_name_ = category_of_points_of_time_[t - 1][similatest_category]->name_;
            category_of_points_of_time_[t][i]->heredity_name_ = category_of_points_of_time_[t - 1][similatest_category]->heredity_name_;
        }
    }
}

void Object_Tracking::estimateTheVelocity()
{
    int t = category_of_points_of_time_.size() - 1;
    if(t < 5) return;

    for(int i = 0;i < category_of_points_of_time_[t].size();i++)
    {
        geometry_msgs::Point observed_value_vel;
        geometry_msgs::Point predicted_value_vel;
        double det_time = category_of_points_of_time_[t][0]->tx_ - category_of_points_of_time_[t - 1][0]->tx_;
        if(det_time < 0.00001) det_time = 0.1;

        int last_name = category_of_points_of_time_[t][i]->last_name_;
        if(last_name == -1)
        {
            category_of_points_of_time_[t][i]->linear_velocity_.x = 0;
            category_of_points_of_time_[t][i]->linear_velocity_.y = 0;
            category_of_points_of_time_[t][i]->linear_velocity_.z = 0;
            continue;
        }
        else
        {
            observed_value_vel.x = (category_of_points_of_time_[t][i]->centrol_x_ - category_of_points_of_time_[t - 1][last_name]->centrol_x_)
                                 / det_time;
            observed_value_vel.y = (category_of_points_of_time_[t][i]->centrol_y_ - category_of_points_of_time_[t - 1][last_name]->centrol_y_)
                                 / det_time;
            observed_value_vel.z = (category_of_points_of_time_[t][i]->centrol_z_ - category_of_points_of_time_[t - 1][last_name]->centrol_z_)
                                 / det_time;
        
            vector<double> v_x;
            vector<double> v_y;
            vector<double> tt;
            int name_ptr = i;
            for(int j = 0;j < category_of_points_of_time_.size();j++)
            {
                //v_x.push_back(category_of_points_of_time_[t - j][name_ptr]->linear_velocity_.x);
                //v_y.push_back(category_of_points_of_time_[t - j][name_ptr]->linear_velocity_.y);
                v_x.push_back(category_of_points_of_time_[t - j][name_ptr]->centrol_x_);
                v_y.push_back(category_of_points_of_time_[t - j][name_ptr]->centrol_y_);
                tt.push_back(category_of_points_of_time_[t - j][0]->tx_);
                name_ptr = category_of_points_of_time_[t - j][name_ptr]->last_name_;
                if(name_ptr == -1) break;
            }
            for(int j = 0;j < tt.size();j++)
            {
                tt[j] -= *(tt.end() - 1);
            }
            
            regressionAnalysis2(predicted_value_vel,v_x,v_y,tt);

            predicted_variance_ = 0;
            category_of_points_of_time_[t][i]->linear_velocity_.x = (predicted_variance_ * predicted_variance_ * observed_value_vel.x + measure_variance_ * measure_variance_ * predicted_value_vel.x) 
                                                                  / (predicted_variance_ * predicted_variance_ + measure_variance_ * measure_variance_);
            category_of_points_of_time_[t][i]->linear_velocity_.y = (predicted_variance_ * predicted_variance_ * observed_value_vel.y + measure_variance_ * measure_variance_ * predicted_value_vel.y) 
                                                                  / (predicted_variance_ * predicted_variance_ + measure_variance_ * measure_variance_);

            //rqtPlotDisplay(category_of_points_of_time_[t][i]->linear_velocity_.x);
        }
    }
    //cout << endl << endl;
}

void Object_Tracking::regressionAnalysis(geometry_msgs::Point& predicted_value_vel,vector<double> v_x,vector<double> v_y,vector<double> tt)  ////v_x,V_y,tt is reverse to time
{

    for(int i = 1;i < tt.size();i++)
    {
        double l = 1;
        for(int j = 1;j < tt.size();j++)
        {
            if(i != j)                  //          -=  += *= /=
            {
                if(tt[i] != tt[j])
                {
                    l *= ((tt[0] - tt[j]) / (tt[i] - tt[j]));
                }
                else
                {
                    cout << "the time difference between two frames of data is wrong." << endl;
                }
            }
        }
        predicted_value_vel.x += (v_x[i] * l);
        predicted_value_vel.y += (v_y[i] * l);
    }
}

void Object_Tracking::regressionAnalysis2(geometry_msgs::Point& predicted_value_vel,vector<double> v_x,vector<double> v_y,vector<double> tt)  ////v_x,V_y,tt is reverse to time
{
    /////// y = a * x + b;
    double a0,a1;
    double b0,b1;
    double item00 = 0 , item01 = 0 , item02 = 0 , item10 = 0 , item11 = 0 ,item12 = 0 , item2 = 0 , item3 = 0;
    double ave_x,ave_y,ave_t;
    if(tt.size() > 5)
    {
        for(int i = 1;i < tt.size();i++)
        {
            ave_x += (v_x[i] / (tt.size() - 1));
            ave_y += (v_y[i] / (tt.size() - 1));
            ave_t += (tt[i] / (tt.size() - 1));
            item00 += (tt[i] * v_x[i]);
            item02 += (v_x[i]);
            item10 += (tt[i] * v_y[i]);
            item12 += (v_y[i]);
            item2 += (tt[i] * tt[i]);
            item3 += (tt[i]);
        }
        a0 = (item00 - item3 * item02 / (tt.size() - 1)) / (item2 - item3 * item3 / (tt.size() - 1));
        a1 = (item10 - item3 * item12 / (tt.size() - 1)) / (item2 - item3 * item3 / (tt.size() - 1));
    }
    else 
    {
        a0 = 0;
        a1 = 0;
    }
    b0 = ave_x - a0 * ave_t;
    b1 = ave_y - a1 * ave_t;

    // predicted_value_vel.x = a0 * tt[0] + b0;
    // predicted_value_vel.y = a1 * tt[0] + b1;
    predicted_value_vel.x = a0;
    predicted_value_vel.y = a1;
}

void Object_Tracking::dynamicCb(point_cloud_test::point_cloud_processe_Config &config,uint32_t level)
{
    cout << "reconfigure request:" << config.cluster_threshold_slope << endl;
    cluster_threshold_slope_ = config.cluster_threshold_slope;
    w1_ = config.similarity_coefficient_1;
    w2_ = config.similarity_coefficient_2;
}

void Object_Tracking::subPointsCb(sensor_msgs::PointCloud msg)
{
    point_lidar_.x = msg.points[0].x;
    point_lidar_.y = msg.points[0].y;
    point_lidar_.z = msg.points[0].z;

    point_cloud_.clear();
    geometry_msgs::Point32 point1;
    for(int i = 1;i < msg.points.size();i++)
    {
        point1.x = msg.points[i].x;
        point1.y = msg.points[i].y;
        point1.z = msg.points[i].z;
        point_cloud_.push_back(point1);
    }
    pointCloudClassification(msg);
}

void Object_Tracking::rqtPlotDisplay(float data)
{
    std_msgs::Float32 data1;
    data1.data = data;
    pub_rqt_plot_.publish(data1);
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "object_tracking");
    cout << "begin to object tracking." << endl;
    Object_Tracking test1;
    return 0;
}