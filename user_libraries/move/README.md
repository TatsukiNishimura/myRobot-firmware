# 電圧と角速度の関係式

LiPo の電圧は 11.3V

<table>
<thead>
<td>左</td>
<td>右</td>
<td>電圧</td>
<td>pwm</td>
</thead>
<tr>
<td>1.19</td>
<td>1.31</td>
<td>1.469</td>
<td>0.13</td>
</tr>
<tr>
<td>2.16</td>
<td>2.1163</td>
<td>1.921</td>
<td></td>
</tr>
<tr>
<td>2.79</td>
<td>2.6388</td>
<td>2.26</td>
<td></td>
</tr>
<tr>
<td>3.73</td>
<td>3.669</td>
<td>2.825</td>
<td></td>
</tr>
<tr>
<td>4.788</td>
<td>4.85</td>
<td>3.39</td>
<td></td>
</tr>
</table>

## 左タイヤ

y は角速度、x は電圧

```
y = 2.3022x -2.53110
```

## 右タイヤ

y は角速度、x は電圧

```
y = 2.2829x -2.5004
```

## 旋回速度

y は旋回角速度、x は電圧

```
y = 0.6374x-0.8083
```
