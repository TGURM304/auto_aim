# commit

## 格式:

```
<type>([scope]): <subject>

[body]

[BREAKING CHANGE: ...]
[footer]
```

### type:

- **feat**: 新功能, 新特性.
- **fix**: 修改 bug.
- **perf**: 在不影响代码内部行为的前提下, 更改代码, 对程序性能进行优化.
- **refactor**: 代码重构, 即在不影响代码内部行为, 功能下的代码修改.
- **docs**: 文档修改.
- **style**: 代码格式修改.
  > ***e.g.***  
  > 添加分号  
  > 修改或添加空格  
  > 更改缩进格式  
- **test**: 测试的新增, 修改.
- **build**: 影响项目构建或依赖项修改.
- **revert**: 恢复上一次提交.
- **ci**: 持续集成相关文件修改.
- **chore**: 其他修改, 即不在上述类型中的修改.
- **release**: 发布新版本.
- **workflow**: 工作流相关文件修改.
- **merge**: 代码合并.

### scope:

scope ⽤于说明 commit 影响的范围，视项⽬不同⽽不同.

### subject:

subject 是 commit ⽬的的简短描述，不超过50个字符.

