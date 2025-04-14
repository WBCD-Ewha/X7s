### Cloth Unfolding Module 
**코드 구성**
- `cloth_unfolding/code/wbcd_cloth.py` 의 `cloth_bimanual_grasp` 함수 이용

**Submodule 초기화 및 업데이트**
```
git submodule init
git submodule update
```

- 다른 팀원이 X7s를 처음 클론받을 경우에도 위 명령어를 통해 submodule을 설정해야 합니다.

**이후 cloth_unfolding에 변경 사항이 생겼을 때 X7s에서 업데이트**

```
cd cloth_unfolding
git checkout main  # 업데이트하고 싶은 브랜치 혹은 태그
git pull           # 새 변경 사항 받아오기
cd ..
git add cloth_unfolding
git commit -m "Update cloth_unfolding submodule to latest"
git push
```
