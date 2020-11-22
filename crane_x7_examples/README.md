[English](README.en.md) | [日本語](README.md)

# crane_x7_examples

炭酸の入ったボトルを振る動作をするコード、ペットボトルを模した円柱のモデルを追加したブランチです。  
  


#  動作環境 
  
以下の環境で動作確認を行っています。  
・ROS Melodic  
　・OS: Ubuntu 18.04.5 LTS  
　・ROS Distribution: Melodic Morenia 1.14.3  
　・Rviz 1.13.13  
　・Gazebo 9.0.0  
   
 ---
 
 
  
###  gazebo上でシミュレーションを行う場合
  
シミュレータを使う場合、授業と同じように以下のコードを実行します。  

```sh
roslaunch shake_bottle_crane_x7_ros crane_x7_with_table.launch
```  
---

###  実機を使う場合  
  
 実機で動作を確認する場合、CRANE_X7からx軸方向に300mm、y軸方向に200mm離れたところにボトルを設置します。  
 (以下の画像はcm表記)  
  
<img src=https://github.com/robotcreating2020-1/images/blob/master/2020-11-15_1.png width=500px />
  
 次に制御信号ケーブルを接続した状態で次のコマンドを実行します。  
 ```sh
 sudo chmod 777 /dev/ttyUSB0  
 roslaunch crane_x7_control crane_x7_control.lanch  
 ```

---

   
## 実行方法  
  
CRANE_X7にボトルを振らせるコードです。  
ボトルを設置した後、crane_x7_examples下にあるbottle_pick.pyを実行します。  
  
```sh
rosrun shake_bottle_crane_x7_ros bottle_pick.py 
```  
  
ボトルは(x,y) = (0.3,0.2)に準備します。  
アームがボトルをつかんだ後、ボトルを6回ほど振り、机に置きます。  
  
ボトルを振るジョイントはfor文の中で直接指定しているので、容易に変更可能です。(デフォルトでは0番と6番を指定)  
また、動かす角度も変えられます。(デフォルトでは0番を30度、6番を55度にしています。)  
  
---
  
### gazebo上での動作  
  
gazebo上での動作は以下のようになります。  
<img src=https://github.com/robotcreating2020-1/images/blob/master/Sim_SoE.gif width=500px />  
  
---
  
### 実機での動作  
  
実機での動作は以下のようになります。  
<img src=https://github.com/robotcreating2020-1/images/blob/master/State_of_Execution.gif width=500px /> 

---
  
### 発表用資料  
  
発表用資料の動画は[こちら](https://youtu.be/MC9XJ4XuVqc)

