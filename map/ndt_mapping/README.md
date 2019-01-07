# Fast NDT SLAM

## 1. 架构

### ndt_mapping

### ndt_matching



## 2. NDT配准的四种库

> #### (1) pcl自带点云库 
>
> `速度太慢,匹配一帧高达3s`
>
> #### (2) ndt_cpu : TODO:尚未实现
>
> #### (3) ndt_gpu
>
> `速度快,100-200ms/帧匹配,但是容易造成现存溢出` ==>> 使用共享内存机制也没有解决问题,需要进一步排查
>
> #### (4) ndt_omp:TODO:尚未实现
>
>

