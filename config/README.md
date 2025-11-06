## CH348 8ch USB Driver 설치방법
1. 다운로드 받은 압축파일을 풀고 생성된 LINUX 디렉토리의 README.md 파일의 내용을 참조해서 Driver를 설치
  * 참고: https://github.com/WCHSoftGroup/ch9344ser_linux/tree/main

```bash
cd /home/temaat/tmcart/config
unzip ch348_linux_driver.zip
```

2. 위 사이트 혹은 README.md 파일내용과 같이 /home/temaat/tmcart/config/LINUX/driver 디렉토리로 이동후 make -> make load -> make install 명령을 차례로 수행
```bash
cd /home/temaat/tmcart/config/LINUX/driver
make
sudo make load
sudo make install
```

3. 컴파일 오류시 --> 제미나이 협조 필요
ch9344.c 파일의 840번째 줄의 ch9344_tty_write 함수의 반환 타입과 세 번째 인자의 타입을 아래와 같이 수정후 재 컴파일
```bash
// static int ch9344_tty_write(struct tty_struct *tty, const unsigned char *buf, int count)
static ssize_t ch9344_tty_write(struct tty_struct *tty, const unsigned char *buf, size_t count)
{
```

## 드라이버 자동 로드 등록
시스템이 시작될 때마다 ch9344 or ch348 커널 모듈(드라이버)을 자동으로 메모리에 올리는 기능
```bash
cd /home/temaat/tmcart/config
# 자동실행 스크립트 파일 등록
sudo cp ch348-install.sh /usr/local/sbin/ch348-install.sh
# 스크립트에 실행 권한 부여
sudo chmod +x /usr/local/sbin/ch348-install.sh 
# systemd 서비스 파일 등록
sudo cp ch348-loader.service /etc/systemd/system/ch348-loader.service

# 서비스 등록 및 활성화
sudo systemctl daemon-reload  # 시스템에 새 서비스 알림
sudo systemctl enable ch348-loader.service # 부팅 시 실행하도록 설정

# 현재 상태에서 즉시 실행 테스트
sudo systemctl start ch348-loader.service
# 결과확인
ls -l /dev/ttyCH*

# 리부팅후 결과확인
sudo reboot
ls -l /dev/ttyCH*
```
