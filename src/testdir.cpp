
#include <boost/filesystem.hpp>
#include <string>
#include <iostream>
 
void resucurePath(boost::filesystem::path src_path, boost::filesystem::path dest_path)
{
  using namespace boost::filesystem;
  if (is_directory(src_path))
  {
    directory_iterator tmp_directory_end;
    directory_iterator tmp_dir_it(src_path);
 
    for (tmp_dir_it; tmp_dir_it != tmp_directory_end; tmp_dir_it++)
    {
      path child_dest_path = dest_path;
      if (is_directory(*tmp_dir_it))
      {
        std::string tmp_dir_name = (*tmp_dir_it).path().filename().string();
        child_dest_path.append(tmp_dir_name.begin(), tmp_dir_name.end());
 
        if (!exists(child_dest_path))
        {
          create_directory(child_dest_path);
        }
        std::cout << child_dest_path << std::endl;
      }
      else
      {
        std::string tmp_dir_name = (*tmp_dir_it).path().stem().string();
        child_dest_path.append(tmp_dir_name.begin(), tmp_dir_name.end());
      }
      resucurePath((*tmp_dir_it).path(), child_dest_path);
    }
  }
  else if (is_regular_file(src_path))
  {
    FILE *fp(NULL);
    fp = fopen(dest_path.string().c_str(), "w+b");
    fclose(fp);
    fp = NULL;
    std::cout <<src_path<<" "<< dest_path << std::endl;
  }
  
}
 
int main()
{
  using namespace boost::filesystem;
  path src_path("/mnt/nvme/once/data/000027");
  path dest_path("/mnt/nvme/once/out");
  resucurePath(src_path, dest_path);
}