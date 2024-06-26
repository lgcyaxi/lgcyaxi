<center>
<p align="center">
  <a href="https://greasyfork.org/zh-CN/scripts/445961-digit77-helper">
    <img width="100" src="https://www.digit77.com/_nuxt/logo-s.BqVYlxIi.png" alt="Digit77">
  </a>
</p>

<h1 align="center">Digit77 Helper</h1>

<div align="center">
  <strong>👉 自动复制下载网址的提取码 自动跳过广告链接的等待时间 网盘自动填写提取码、自动下载、自动保存！
   👈</strong><br>
  <sub>适用于<a herf="https://www.digit77.com/">Digit77.com Mac精品应用免费分享网</a>的油猴脚本</sub>
</div>

**Digit77网页助手油猴脚本**。自动复制下载网址的提取码 自动跳过广告链接的等待时间 网盘自动填写提取码、自动下载、自动保存！

[在greasyfork查看](https://greasyfork.org/zh-CN/scripts/495107-digit77-helper)

## 💽 安装地址

- [**greasyfork下载地址（如果网络支持，更新最快）**](https://greasyfork.org/zh-CN/scripts/495107-digit77-helper)
- [github下载地址](https://github.com/lgcyaxi/lgcyaxi/raw/main/projs/digit77helper/Digit77Helper.user.js)

## 🔧 助手配置

进入[Digit77.com Mac精品应用免费分享网](https://www.digit77.com/)后，在软件下载页面的下载框后出现一个配置选项，点击即可进入设置：（注意翻到底部点击保存设置！）

<img src="README/image-20240515.png" alt="image-20220605153852187" style="zoom:50%;" />

## 💯 常见问题

💡 **总是碰到一个用户脚本试图访问跨园资源的提示？**

<img src="README/image-20220605155346852.png" alt="image-20220605155346852" style="zoom:25%;" />

A：为了能无需通过用户剪贴板进行提取码的传输，我们需要获取网盘的链接，从而模仿ouo这个广告链接的协议请求真正的网盘链接来进行提取码的传递，**建议点击 总是允许此域名/总是允许全部域名 来防止油猴插件多次弹出提醒**。如果拒绝，脚本会自动复制提取码到剪贴板，请自行粘贴提取码。

💡 **助手安全吗？**

A：助手免费开源，代码均在本地运行，获Digit77 Helper站长推荐。

## 👻 BUG反馈

如果您在使用过程中有无法识别的文本，请 [在GitHub提交issues](https://github.com/lgcyaxi/lgcyaxi/issues) 进行反馈。

## 📜ToDo

- [X] 自动等待ouo/cloaking并跳转
- [X] 实现OneDrive自动填写提取码
- [X] 实现夸克网盘自动填写提取码
- [ ] 优化提取码存储

## 📖 更新日志

**v2.4.5** 重新支持digit77的夸克网盘/OneDrive的验证码提取, 合并了设置界面，移除了不适配的代码

## 🫶🏼 特别鸣谢

感谢原作者[XYZliang](https://github.com/XYZliang/) 对脚本的无私贡献和支持

感谢Digit77.com站长的公益分享和对本插件的支持

感谢[网盘智能识别助手](https://github.com/syhyz1990/panAI)，引用部分代码实现对天翼和阿里网盘的支持

感谢[凯速网](https://my.ksust.com/kstore.htm)提供的[免费网盘和静态资源储存](https://my.ksust.com/kstore.htm?aff=2078)，实现设置页面和全球高速脚本下载

感谢千牛云提供的OSS和全球CDN作为备用下载
