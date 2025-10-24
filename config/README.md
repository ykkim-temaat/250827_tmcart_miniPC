## CH348 8ch USB Driver 설치방법
1. 다운로드 받은 압축파일을 풀고 생성된 LINUX 디렉토리의 README.md 파일의 내용을 참조해서 Driver를 설치
  * 참고: https://github.com/WCHSoftGroup/ch9344ser_linux/tree/main

## 드라이버 자동 로드 등록
$ echo "ch9344" | sudo tee /etc/modules-load.d/ch9344.conf
 --> 설명: 위 명령어는 앞으로 시스템이 시작될 때마다 ch9344 커널 모듈(드라이버)을 자동으로 메모리에 올리도록 설정 파일을 생성하는 것입니다.