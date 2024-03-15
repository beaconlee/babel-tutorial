#include "iostream"
#include "src/common_msgs/user/user_data.pb.h"


int main()
{
  data::users::UserProfile userProfile;
  userProfile.set_email("756633687@qq.com");
  userProfile.set_id(1);

  std::cout << "user profile id: " << userProfile.id()
            << " email: " << userProfile.email() << "\n";

  return 0;
}